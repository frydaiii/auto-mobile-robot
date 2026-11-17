"""Only this node forwards user commands to the simulated drive."""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile

from amr_bringup.command_gate import CommandGate


class CommandGateNode(Node):
    def __init__(self):
        super().__init__('command_gate')
        self.gate = CommandGate(**{
            name: self.declare_parameter(name, default).value
            for name, default in (
                ('linear_limit', 0.15), ('angular_limit', 0.6), ('timeout', 0.5))
        })
        qos = QoSProfile(depth=1)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_safe', qos)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.receive, qos)
        # Expire even while Gazebo is paused. ROS nodes still use sim time.
        self.timer = self.create_timer(
            0.02, self.publish_command, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, message):
        self.gate.receive(message.linear.x, message.angular.z, time.monotonic())

    def publish_command(self):
        message = Twist()
        message.linear.x, message.angular.z = self.gate.output(time.monotonic())
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CommandGateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
