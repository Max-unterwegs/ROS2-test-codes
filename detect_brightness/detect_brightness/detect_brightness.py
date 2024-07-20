import rclpy, cv2, numpy
from sensor_msgs.msg import Image, CompressedImage,CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8, Float64
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile
import math
bridge = CvBridge()  # 转换为ros2的消息类型(imgmsg)的工具
import numpy as np
import time

class DetectBrightness(Node):
    def __init__(self):
        super().__init__('detect_brightness')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/oakd/rgb/preview/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=1))
        self.publisher = self.create_publisher(Float64, '/brightness_topic', 5)
        self.bridge = CvBridge()
        self.cv_image = None

        
        self.sub_image_type = 'compressed'
        self.pub_image_type = 'compressed'
        
        if self.sub_image_type == "compressed":
            # subscribes compressed image
            self.sub_image_original = self.create_subscription(CompressedImage, '/oakd/rgb/preview/image_raw/compressed',  self.image_callback, QoSProfile(depth=1))
        elif self.sub_image_type == "raw":
            # subscribes raw image
            self.sub_image_original = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, QoSProfile(depth=1))
        #self.get_logger().info('start create publisher image')
        # if self.pub_image_type == "compressed":
        #     # publishes compensated image in compressed type 
        #     self.pub_bright = self.create_publisher(CompressedImage, '/brightness_topic',  QoSProfile(depth=1))
        # elif self.pub_image_type == "raw":
        #     # publishes compensated image in raw type
        #     self.pub_bright = self.create_publisher(Image, '/brightness_topic', QoSProfile(depth=1))
        # #self.get_logger().info('start create publisher light')


    def image_callback(self, image_msg):
       
        try:
            if self.sub_image_type == "compressed":
                #np_arr = np.fromstring(image_msg.data, np.uint8)
                np_arr = np.frombuffer(image_msg.data,np.uint8) 
                self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
            return

        brightness = self.calculate_brightness(self.cv_image)
        brightness_msg = Float64()
        brightness_msg.data = brightness
        self.get_logger().info("Bright:%f"% brightness)
        self.publisher.publish(brightness_msg)

    def calculate_brightness(self, image):
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        brightness = np.mean(gray_image)
        return brightness


def main(args=None):
    rclpy.init(args=args)
    detect_brightness = DetectBrightness()
    rclpy.spin(detect_brightness)
    detect_brightness.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()