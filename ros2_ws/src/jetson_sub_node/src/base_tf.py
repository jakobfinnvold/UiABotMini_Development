#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Twist, Quaternion
from nav_msgs.msg import Odometry

from math import cos, sin


class tf2Base(Node):

    def __init__(self):
        super().__init__("tf2Base")

        # -- Subscribe to Odom topic from Jetson -- #
        self.odom_sub = self.create_subscription(JointState, "joint_states", self.odom_callback, 10)

        # -- Variables for transform publish -- #
        self.tf_broadcaster_r = tf2_ros.TransformBroadcaster(self)
        self.tf_broadaster_l = tf2_ros.TransformBroadcaster(self)
        self.right_frame = "right_wheel"
        self.left_frame = "left_wheel"

        #self.odom_frame_id = "odom"
        self.base_frame_id = "base_link"

        # -- General Variables -- #
        #### TWIST ####
        self.linear_x = None
        self.linear_y = None
        self.angular_z = None
        self.right = 0.0
        self.left = 0.0
        self.v_r = 0.0
        self.v_l = 0.0
        #### POSE ####
        self.pose_x = None
        self.pose_y = None
        self.pose_orient_z = None
        self.pose_orient_w = None

        self.sub1 = self.create_subscription(Twist, "wheels", self.cv1, 10)

    def cv1(self, msg):

        self.v_r = msg.angular.x 
        self.v_l = msg.angular.y 

    # def odom_callback(self, msg):
    #     t = TransformStamped()
    #     self.pose_x = msg.pose.pose.position.x  
    #     self.pose_y = msg.pose.pose.position.y 
    #     self.pose_orient_z = msg.pose.pose.orientation.z 
    #     self.pose_orient_w = msg.pose.pose.orientation.w 

    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = self.odom_frame_id
    #     t.child_frame_id = self.base_frame_id
    #     t.transform.translation.x = self.pose_x
    #     t.transform.translation.y = self.pose_y
    #     t.transform.translation.z = 0.0
    #     t.transform.rotation.x = 0.0
    #     t.transform.rotation.y = 0.0
    #     t.transform.rotation.z = self.pose_orient_z
    #     t.transform.rotation.w = self.pose_orient_w

    #     self.tf_broadcaster_odom.sendTransform(t)

    def odom_callback(self, msg):
        self.left, self.right = msg.position

        t_r = TransformStamped()
        t_l = TransformStamped()

        t_r.header.stamp = self.get_clock().now().to_msg()
        t_l.header.stamp = self.get_clock().now().to_msg()
        t_r.header.frame_id = self.base_frame_id
        t_l.header.frame_id = self.base_frame_id
        t_r.child_frame_id = self.right_frame
        t_l.child_frame_id = self.left_frame
        t_r.transform.translation.x = self.right*0.01
        t_l.transform.translation.x = self.left*0.01
        t_r.transform.translation.y = -0.08
        t_l.transform.translation.y = 0.08
        t_r.transform.translation.z = -0.082
        t_l.transform.translation.z = -0.082
        t_r.transform.rotation.x = 3.14159265359/2
        t_l.transform.rotation.x = -3.14159265359/2
        t_r.transform.rotation.z = self.right/0.01
        t_l.transform.rotation.z = self.left/0.01

        self.tf_broadcaster_r.sendTransform(t_r)
        self.tf_broadaster_l.sendTransform(t_l)


def main():
    rclpy.init()
    node = tf2Base()
    try:
        rclpy.spin(node) # Spin it
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()