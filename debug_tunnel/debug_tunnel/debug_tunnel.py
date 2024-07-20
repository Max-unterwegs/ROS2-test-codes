import rclpy, cv2, numpy
from sensor_msgs.msg import Image, CompressedImage,CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8, Float64
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile
import math
import time
import threading
bridge = CvBridge()  # 转换为ros2的消息类型(imgmsg)的工具
import numpy as np
import time

class Tunnel(Node):
    def __init__(self):
        super().__init__('debug_tunnel')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 5)
        self.twist = Twist()
    def pub_cmd_vel(self):
        self.twist.angular.z = 0.0
        self.twist.linear.x = 1.0
        time.sleep(2)
        running_cmd_vel(self.running, self.twist, 6)
        time.sleep(1)
        # 2.转弯
        self.twist.angular.z = -1.45  # 1.57是90度,1.57 rad/s
        self.twist.linear.x = 0.0    # 加上直行实现转弯
        running_cmd_vel(self.running, self.twist, 1)
        time.sleep(1)
        # 3.直行
        self.twist.angular.z = 0.0
        self.twist.linear.x = 1.0
        running_cmd_vel(self.running, self.twist, 2)
    def running(self,twist, stop_event):
        while not stop_event.is_set():
        #这里直接发布运动指令
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(0.1)

class StoppableThread(threading.Thread):
    def __init__(self, target, *args, stop_event, **kwargs):
        super().__init__(target=target, *args, **kwargs)
        self.stop_event = stop_event

    def stop(self):
        self.stop_event.set()
        
    def start(self):
        self.stop_event.clear()
        super().start()

def running_cmd_vel(function,twist,sec):
    stop_event = threading.Event()  # Define the stop_event variable
    t = StoppableThread(target=function, args=(twist, stop_event),stop_event=stop_event)
    t.start()
    time.sleep(sec)
    if t.is_alive():
        print("Stopping thread")
        t.stop()
    t.join()
    print("Finished")
    
def main(args=None):
    rclpy.init(args=args)

    Tunnel_node = Tunnel()

    Tunnel_node.pub_cmd_vel()
    # rclpy.spin(Tunnel_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Tunnel_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()


