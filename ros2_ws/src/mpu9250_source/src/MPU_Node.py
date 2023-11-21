#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu

import serial
import struct
from time import sleep


## ----- Defining parameters ------ ##
global ser 
ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)
ser.close()
ser.open()

class Pernille(Node):

    def __init__(self):
        super().__init__("imu_publisher")
        self.publisher_ = self.create_publisher(Imu,'imu/data', 10)
        timer_period = 0.006
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.imu_name = self.declare_parameter('~imu_frame', 'imu_link').get_parameter_value().string_value


    def timer_callback(self):
        
        read_val = ser.read(size=14)
        unpacked = struct.unpack("=hfff", read_val)
        imu_msg = Imu()
        imu_msg.header.frame_id = "imu_link"
        header, xAccel, zAngular, yaw = unpacked

        if header == 300:
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.linear_acceleration.x = xAccel
            imu_msg.angular_velocity.z = zAngular
            imu_msg.orientation.z = yaw
            imu_msg.orientation.w = 1.0

            self.publisher_.publish(imu_msg)
            #self.get_logger().info(" I published: [%f, %f, %f, %f]"%(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w))


def main(args=None):
    rclpy.init(args=args)
    node = Pernille()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':

    main()
