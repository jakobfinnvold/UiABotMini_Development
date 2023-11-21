#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
from math import cos, sin

NS_TO_SEC= 1000000000

class tf2Wheel(Node):

    def __init__(self):
        super().__init__("tf2Wheel")
        ### --- Subscribing to wheel topics --- ###
        # self.l_wheel_sub = self.create_subscription(Twist, "lwheel", self.lwheel_callback, 10)
        # self.r_wheel_sub = self.create_subscription(Twist, "rwheel", self.rwheel_callback, 10)
        self.wheel_sub = self.create_subscription(Twist, "wheels", self.wheels_callback, 10)
        
        ### --- TF broadcaster variables --- ###
        # self.tf_broadcaster_odom = tf2_ros.TransformBroadcaster(self)
        # self.tf_l_broadcaster = tf2_ros.TransformBroadcaster(self)
        # self.tf_r_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.base_frame_id = self.declare_parameter('base_frame_id', 'base_link').value
        self.odom_frame = self.declare_parameter('odom_frame_id', 'odom').value
        #self.r_frame = 'right_wheel'

        ### --- Initialization of parameters --- ###
        self.l_wheel_speed = 0.0
        self.r_wheel_speed = 0.0
        self.w_speed = 0.0
        self.update_frequency = 0.01 #1/30Hz
        self.then = self.get_clock().now()
        self.wheel_radius = 0.068/2
        self.wheel_distance = 0.2
        self.theta = 0.0
        self.robot_vx = 0.0
        self.robot_vy = 0.0
        self.robot_w = 0.0
        self.p_l = 0.0
        self.p_r = 0.0
        self.time_callback = self.get_clock().now()
        self.joint_time = 0.0

        ### --- Setting up publisher to /wheel/odometry --- ###
        self.odom_publisher_ = self.create_publisher(Odometry, "/wheel/odometry", 10)
        self.create_timer(self.update_frequency, self.update)

        self.joint_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(self.update_frequency, self.get_states)


    def wheels_callback(self, msg):
        # self.r_wheel_speed = float(msg.twist.twist.linear.x)
        # self.l_wheel_speed = float(msg.twist.twist.linear.y)
        self.r_wheel_speed = float(msg.angular.x) 
        self.l_wheel_speed = float(msg.angular.y) 

    def update(self):
        

        mat1 = np.matrix([[self.wheel_radius/2, self.wheel_radius/2], 
                          [0, 0], 
                          [self.wheel_radius/(2*self.wheel_distance), -self.wheel_radius/(2*self.wheel_distance)]])
        
        mat2 = np.matrix([[self.r_wheel_speed], [self.l_wheel_speed]])

        self.theta = 0 + self.robot_w #(self.r_wheel_speed - self.l_wheel_speed)/self.wheel_distance

        T1=np.matrix([[cos(self.theta), sin(self.theta),0],[-sin(self.theta),cos(self.theta),0], [0,0,1]])
    
        m =np.matmul(mat1,mat2)
        m_m = np.matmul(T1,m)

        self.robot_vx= float(m_m[0])
        self.robot_vy = float(m_m[1])
        self.robot_w = float(m_m[2])
        now = self.get_clock().now()
        
        ### --- Publishing Odometry calculated from Wheels --- ###
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.robot_vx
        odom.twist.twist.linear.y = self.robot_vy
        odom.twist.twist.angular.z = self.robot_w*-2
        self.odom_publisher_.publish(odom)

    def get_states(self):
        self.now = self.get_clock().now()
        self.p_l = self.p_l + self.l_wheel_speed*0.01
        self.p_r = (self.p_r + self.r_wheel_speed*0.01)
        
        joint_message = JointState()
        joint_message.header.stamp = self.now.to_msg()
        joint_message.name = ["left_wheel_joint", "right_wheel_joint"]
        joint_message.position = [self.p_l, self.p_r]     
        self.joint_publisher.publish(joint_message)


def main():
    rclpy.init()
    node = tf2Wheel()
    try:
        rclpy.spin(node) # Spin it
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
