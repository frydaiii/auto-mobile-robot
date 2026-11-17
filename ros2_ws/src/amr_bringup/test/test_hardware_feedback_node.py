"""Real ROS boundary checks for raw sensor semantics and contacts."""
import math
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu, JointState, LaserScan, Range
from ros_gz_interfaces.msg import Contact, Contacts
from std_msgs.msg import Bool, Int64MultiArray

from amr_bringup.hardware_feedback_node import HardwareFeedbackNode


def test_ros_adapters_publish_raw_imu_range_ticks_and_contacts():
    rclpy.init()
    node = HardwareFeedbackNode()
    peer = rclpy.create_node('feedback_test_peer')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    received = {name: [] for name in ('imu', 'range', 'ticks', 'bumper')}
    subs = [peer.create_subscription(kind, topic, received[key].append, 10)
            for kind, topic, key in (
                (Imu, '/imu/data_raw', 'imu'),
                (Range, '/range/left', 'range'),
                (Int64MultiArray, '/encoder_ticks', 'ticks'),
                (Bool, '/bumper/left', 'bumper'))]
    pubs = {key: peer.create_publisher(kind, topic, 10) for key, kind, topic in (
        ('imu', Imu, '/sim/imu'), ('range', LaserScan, '/sim/tof/left'),
        ('ticks', JointState, '/joint_states'),
        ('bumper', Contacts, '/sim/bumper/left'))}

    def until(predicate):
        end = time.monotonic()+5
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), 'Feedback ROS condition timed out'

    try:
        until(lambda: all(p.get_subscription_count() for p in pubs.values())
              and all(s.get_publisher_count() for s in subs))
        imu = Imu()
        imu.header.frame_id = 'imu_link'
        imu.orientation.w = 1.0
        imu.angular_velocity.z = 0.2
        imu.linear_acceleration.z = 9.81
        pubs['imu'].publish(imu)
        scan = LaserScan()
        scan.header.frame_id = 'tof_left_link'
        scan.range_min, scan.range_max = 0.03, 2.0
        scan.angle_min, scan.angle_max = -0.218166, 0.218166
        scan.ranges = [math.inf, 0.4, 1.0]
        pubs['range'].publish(scan)
        joints = JointState()
        joints.header.stamp.sec = 1
        joints.name = ['left_wheel_joint', 'right_wheel_joint']
        joints.position = [0.0, 0.0]
        pubs['ticks'].publish(joints)
        contact = Contacts()
        contact.contacts = [Contact()]
        pubs['bumper'].publish(contact)
        until(lambda: all(received.values()))
        assert received['imu'][-1].orientation_covariance[0] == -1
        assert received['imu'][-1].angular_velocity.z == 0.2
        assert received['imu'][-1].linear_acceleration.z == 9.81
        assert received['range'][-1].range == pytest.approx(0.4)
        assert received['range'][-1].field_of_view == pytest.approx(math.radians(25), abs=1e-5)
        assert received['range'][-1].radiation_type == Range.INFRARED
        assert list(received['ticks'][-1].data) == [0, 0]
        assert received['ticks'][-1].layout.dim[0].label == 'left_wheel_joint,right_wheel_joint'
        assert received['bumper'][-1].data
        pubs['bumper'].publish(Contacts())
        until(lambda: not received['bumper'][-1].data)
        joints.header.stamp.sec = 2
        joints.position = [2*math.pi, -math.pi]
        pubs['ticks'].publish(joints)
        until(lambda: list(received['ticks'][-1].data) == [1440, -720])
    finally:
        executor.shutdown()
        peer.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()
