"""Expose native simulated sensors with the robot hardware's ROS semantics."""
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Imu, JointState, LaserScan, Range
from std_msgs.msg import Bool, Int64MultiArray, MultiArrayDimension

from amr_bringup.hardware_feedback import EncoderTicks, nearest_range


class HardwareFeedbackNode(Node):
    def __init__(self):
        super().__init__('hardware_feedback')
        self.encoder = EncoderTicks(self.declare_parameter(
            'encoder_counts_per_revolution', 1440).value)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.encoder_pub = self.create_publisher(Int64MultiArray, '/encoder_ticks', 10)
        self.range_pubs = {side: self.create_publisher(Range, f'/range/{side}', 10)
                           for side in ('left', 'right')}
        self.bumper_pubs = {side: self.create_publisher(Bool, f'/bumper/{side}', 10)
                            for side in ('left', 'center', 'right')}
        self.inputs = [
            self.create_subscription(Imu, '/sim/imu', self.imu, qos_profile_sensor_data),
            self.create_subscription(JointState, '/joint_states', self.joints,
                                     qos_profile_sensor_data),
        ]
        for side in self.range_pubs:
            self.inputs.append(self.create_subscription(
                LaserScan, f'/sim/tof/{side}', partial(self.tof, side),
                qos_profile_sensor_data))
        for side in self.bumper_pubs:
            self.inputs.append(self.create_subscription(
                Contacts, f'/sim/bumper/{side}', partial(self.contact, side),
                qos_profile_sensor_data))

    def imu(self, message):
        # An MPU6050 is an accelerometer + gyro; it does not measure orientation.
        message.orientation.x = message.orientation.y = message.orientation.z = 0.0
        message.orientation.w = 1.0
        message.orientation_covariance = [-1.0] + [0.0]*8
        self.imu_pub.publish(message)

    def joints(self, message):
        stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        ticks = self.encoder.update(message.name, message.position, stamp)
        if ticks is None:
            return
        result = Int64MultiArray()
        result.layout.dim = [MultiArrayDimension(
            label=','.join(self.encoder.names), size=2, stride=2)]
        result.data = list(ticks)
        self.encoder_pub.publish(result)

    def tof(self, side, message):
        result = Range()
        result.header = message.header
        result.radiation_type = Range.INFRARED
        result.field_of_view = message.angle_max - message.angle_min
        result.min_range, result.max_range = message.range_min, message.range_max
        result.range = nearest_range(message.ranges, result.min_range, result.max_range)
        self.range_pubs[side].publish(result)

    def contact(self, side, message):
        self.bumper_pubs[side].publish(Bool(data=bool(message.contacts)))


def main(args=None):
    rclpy.init(args=args)
    node = HardwareFeedbackNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
