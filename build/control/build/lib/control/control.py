import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import math
import os
import cv2
from enum import Enum
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class control(Node):
    def __init__(self):
            super().__init__('control')
            self.control_parking = self.create_subscription(UInt8, '/control/parking', self.control_parking_callback,QoSProfile(depth=1))
            self.paring_feedback = self.create_publisher(UInt8, '/control/parking_feedback', QoSProfile(depth=1))
            self.parkinng_signal = 0

    def control_parking_callback(self, msg):
        if(msg.data == 0 and self.parkinng_signal == 0):
            print('Parking')
            self.paring_feedback.publish(UInt8(data=1))  # //* 1 表示正在停车
            os.system('ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance:0.5,max_translation_speed: 0.3}"')
            os.system('ros2 action send_goal /drive_angle irobot_create_msgs/action/DriveAngle "{angle:-1.57,max_rotation_speed: 0.3}"')
            os.system('ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance:0.5,max_translation_speed: 0.3}"')
            os.system('sleep 2')
            os.system('ros2 action send_goal /drive_angle irobot_create_msgs/action/DriveAngle "{angle:3.14,max_rotation_speed: 0.3}"')
            os.system('ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance:0.2,max_translation_speed: 0.3}"')
            self.get_logger().info("Parking completed!!")
            self.parking_signal = 1
            self.paring_feedback.publish(UInt8(data=0))  # //* 0 表示停车完成
            

def main():
    rclpy.init()
    control_node = control()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
