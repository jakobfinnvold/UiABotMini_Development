#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
import serial
import struct
import os
from math import cos, sin

#ser = serial.Serial('/dev/tty/USB0', baudrate=152000, timeout=0.1)

class FwdKinematics(Node):

    def __init__(self):
        super().__init__("kinematics")
        self.pub = self.create_publisher(Float32, "right_wheel", 10)
        self.pub1 = self.create_publisher(Float32, "left_wheel", 10)
        self.timer = self.create_timer(0.005, self.speed_callback)
        self.timer1 = self.create_timer(0.005, self.speed1_callback)

        self.header = None
        self.right = None
        self.left = None
        self.unpacked = None
        self.x = 0.0
        self.theta = 0.0
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.r = 0.068/2
        self.d = 0.2

        self.velocity_subscriber = self.create_subscription(Twist, "cmd_vel", self.kin_callback, 10)


    def kin_callback(self, msg):
        self.x = msg.linear.x
        self.theta = msg.angular.z*-1
        
        mat1 = np.matrix([[1/self.r, 0, self.d/self.r], [1/self.r, 0, -self.d/self.r]])
        mat2 = np.matrix([[self.x], [0], [self.theta]])

        M = np.matmul(mat1, mat2)
        self.right_speed=M[0]
        self.left_speed=M[1]

        

    def speed_callback(self):
        vel_msg = Float32()
        vel_msg.data= float(self.right_speed)
        self.pub.publish(vel_msg)

    def speed1_callback(self):
        vel_msg1 = Float32()
        vel_msg1.data = float(self.left_speed)
        self.pub1.publish(vel_msg1)

def main():
    rclpy.init()
    node = FwdKinematics() # Init the node   
    rclpy.spin(node) # Spin it
    rclpy.shutdown()

if __name__ == '__main__':    
    main()


