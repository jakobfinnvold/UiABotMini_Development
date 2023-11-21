#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry

import numpy as np
import serial
import struct
import os
from math import cos, sin

os.system('sudo chmod 777 /dev/ttyUSB0')
## ---- Variables to hold the velocity messages ---- ##
global x_velocity
global theta_velocity
global ser
global v_x, v_y, rot

ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=0.1)
## ---- Msg variables are float 32 -> 32 bits -> 4 bytes to send per float value ---- ## 

## -- Function for doing fwd kin -- ##
def forward_kin(right, left):
    r = 0.068/2
    d = 0.2
    mat1 = np.matrix([[r/2, r/2], [0, 0], [r/(2*d), -r/(2*d)]])
    mat2 = np.matrix([[right], [left]])

    theta = (right-left)/d

    T1=np.matrix([[cos(theta), sin(theta),0],[-sin(theta),cos(theta),0], [0,0,1]])
    
    m =np.matmul(mat1,mat2)
    m_m = np.matmul(T1,m)

    return m_m

class Velocity(Node):

    def __init__(self):
        super().__init__("velocity")
        ### ---- Publisher ---- ###
        self.wheel_publisher = self.create_publisher(Twist, "wheels", 10)
        self.timer = self.create_timer(1/50, self.wheel_callback)

        # self.publisher_l = self.create_publisher(Twist, "lwheel", 10)
        # self.publisher_r = self.create_publisher(Twist, "rwheel", 10)
        global head1, right, left, unpacked
        self.head1 = None
        self.right = None
        self.left = None
        #self.publisher_ = self.create_publisher(JointState, "wheels", 10)

        # timer_period = 0.006
        # self.timer = self.create_timer(timer_period, self.left_wheel_callback)
        # self.timer1 = self.create_timer(timer_period, self.right_wheel_callback)

        ### ---- Subscriber ---- ###
        self.velocity_subscriber = self.create_subscription(Twist, "cmd_vel", self.vel_callback, 10)

    ### --- Subscriber callback --- ### (Only writes, and the publisher callback handles the reading)
    
    def vel_callback(self, msg):
        
        x_velocity = msg.linear.x
        theta_velocity = msg.angular.z
        head = 36

        bytePack = (struct.pack("=Bff", head, x_velocity, theta_velocity))
        ser.write(bytePack)

    def wheel_callback(self):
        wheel_speeds = Twist()
        try:
            read_val = ser.read(size=9)
            unpacked = struct.unpack("=Bff", read_val)
            self.head1, self.right, self.left = unpacked

            if self.head1 == 36:
                wheel_speeds.angular.x = self.right
                wheel_speeds.angular.y = self.left
                
                self.wheel_publisher.publish(wheel_speeds)

        except struct.error as err:
            print(err)        

        

    # def left_wheel_callback(self):

    #     try:
    #         read_val = ser.read(size=9)
    #         unpacked = struct.unpack("=Bff", read_val)
    #         self.head1, self.right, self.left = unpacked
    #         #self.get_logger().info("I heard: [%f]", (right))

    #         l_vel_msg = Twist()
    #         # wheel_msg = JointState()
    #         # wheel1_msg = JointState()

    #         if self.head1 == 36:

    #             ## --- Publish message --- ##
    #             l_vel_msg.angular.z = float(self.left)
    #             self.publisher_l.publish(l_vel_msg)
    #     except struct.error as err:
    #         print(err)

    # def right_wheel_callback(self):
    #     r_vel_msg = Twist()

    #     if self.head1 == 36:
    #         r_vel_msg.angular.z = float(self.right)
    #         self.publisher_r.publish(r_vel_msg)


        


def vel_node_init():
    rclpy.init()
    node = Velocity() # Init the node   
    rclpy.spin(node) # Spin it
    rclpy.shutdown()


if __name__ == '__main__':    
    vel_node_init()


    

        