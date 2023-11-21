#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf_imu_broadcaster
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

class tf2IMU(Node): 

    def __init__(self):
        super().__init__("tf2IMU")
        self.imu_name = self.declare_parameter('~imu_frame', 'imu_link').get_parameter_value().string_value
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.imu_sub_ = self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        self.imu_sub_

    def imu_callback(self, msg):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "imu_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0455
        t.transform.rotation.x = msg.orientation.y * 3.1415/180 # Compute radians
        t.transform.rotation.y = msg.orientation.x * 3.1415/180
        t.transform.rotation.z = msg.orientation.z * 3.1415/180
        t.transform.rotation.w = msg.orientation.w
        self.tf_broadcaster.sendTransform(t)

def imu_node():
    rclpy.init()
    node = tf2IMU() # Init the node   
    try:
        rclpy.spin(node) # Spin it
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    imu_node()

