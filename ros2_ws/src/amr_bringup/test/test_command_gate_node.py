"""Exercise real ROS publishers, subscriptions, timers and shutdown paths."""
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy._rclpy_pybind11 import InvalidHandle
from rclpy.parameter import Parameter

from amr_bringup import command_gate_node


def test_ros_gate_expires_without_sim_clock_and_recovers():
    rclpy.init()
    gate = command_gate_node.CommandGateNode()
    peer = rclpy.create_node('gate_test_peer')
    gate.set_parameters([Parameter('use_sim_time', value=True)])
    executor = SingleThreadedExecutor()
    executor.add_node(gate)
    executor.add_node(peer)
    received = []
    subscription = peer.create_subscription(
        Twist, '/cmd_vel_safe', received.append, 10)
    publisher = peer.create_publisher(Twist, '/cmd_vel', 10)
    bumper_publishers = {s: peer.create_publisher(Bool, f'/bumper/{s}', 10)
                         for s in ('left', 'center', 'right')}
    bumper_states = {s: False for s in bumper_publishers}
    bumper_timer = peer.create_timer(0.05, lambda: [
        p.publish(Bool(data=bumper_states[s])) for s, p in bumper_publishers.items()])

    def until(predicate, timeout=5.0):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), 'ROS gate condition timed out'

    def send(linear, angular):
        message = Twist()
        message.linear.x, message.angular.z = linear, angular
        # Unsupported axes must not leak through to differential drive.
        message.linear.y = 7.0
        publisher.publish(message)

    try:
        until(lambda: publisher.get_subscription_count() == 1
              and subscription.get_publisher_count() == 1)
        until(lambda: bool(received))
        assert received[-1] == Twist()
        command_timer = peer.create_timer(0.05, lambda: send(2.0, -3.0))
        until(lambda: received[-1].linear.x > 0)
        command_timer.cancel()
        assert received[-1].linear.x == pytest.approx(0.1418080017245393)
        assert received[-1].angular.z == pytest.approx(-0.5672320068981572)
        assert received[-1].linear.y == 0.0
        until(lambda: received[-1] == Twist(), timeout=2.0)
        # No /clock publisher: ROS time stays paused, steady watchdog still fires.
        assert gate.get_clock().now().nanoseconds == 0
        send(-0.1, 0.2)
        until(lambda: received[-1].linear.x == -0.1)
        send(float('nan'), 0.1)
        until(lambda: received[-1] == Twist())
        send(0.1, 0.0)
        until(lambda: received[-1].linear.x == 0.1)
        bumper_states['center'] = True
        until(lambda: received[-1] == Twist())
        send(0.1, 0.0)
        bumper_states['center'] = False
        # Wait for a clear feedback callback, then require a new command.
        cleared = []
        clear_subscription = peer.create_subscription(
            Bool, '/bumper/center', lambda m: cleared.append(not m.data), 10)
        until(lambda: bool(cleared) and cleared[-1])
        assert received[-1] == Twist()
        send(0.1, 0.0)
        until(lambda: received[-1].linear.x == 0.1)
        bumper_timer.cancel()
        # Keep commands fresh while feedback expires, with sim time still paused.
        active_timer = peer.create_timer(0.05, lambda: send(0.1, 0.0))
        until(lambda: received[-1] == Twist(), timeout=2.0)
        active_timer.cancel()
    finally:
        executor.shutdown()
        peer.destroy_node()
        gate.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize('exit_mode', ['return', 'keyboard', 'external'])
def test_entrypoint_cleans_up_ros_context(monkeypatch, exit_mode):
    seen = []

    def spin(node):
        seen.append(node)
        # The actual node has initialized and can publish before shutdown.
        node.publish_command()
        if exit_mode == 'keyboard':
            raise KeyboardInterrupt
        if exit_mode == 'external':
            rclpy.shutdown()
            raise ExternalShutdownException

    # Replace only the indefinite wait; use real ROS initialization and cleanup.
    monkeypatch.setattr(command_gate_node.rclpy, 'spin', spin)
    try:
        command_gate_node.main(args=[])
        assert len(seen) == 1
        assert not rclpy.ok()
        with pytest.raises(InvalidHandle):
            with seen[0].handle:
                pass
    finally:
        rclpy.try_shutdown()
