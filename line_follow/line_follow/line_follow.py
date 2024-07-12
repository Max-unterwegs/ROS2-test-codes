#!/usr/bin/env python3
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
class LineFollower(Node):
  def __init__(self):
    super().__init__('line_follow')
    self.image_sub = self.create_subscription(CompressedImage, '/oakd/rgb/preview/image_raw/compressed', self.image_callback, QoSProfile(depth=1))
    #self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)s 
    self.image_sub
    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 5)
    self.twist = Twist()
    self.angleBuffer = []
    self.maxmax = 0.0
    self.max_detect_level = 0.0
    self.colorLower = None
    self.colorUpper = None
    self.declare_parameter("~h_min", 25)
    self.declare_parameter("~h_max", 55)
    self.declare_parameter("~s_min", 40)
    self.declare_parameter("~s_max", 255)
    self.declare_parameter("~v_min", 152)
    self.declare_parameter("~v_max", 255)
    self.declare_parameter("~run", 0)
    self.declare_parameter("~x_speed_10", 5)
    self.declare_parameter("~err_grenze", 5)
    self.declare_parameter("~z_speed", 90)
    self.declare_parameter("~zwx_10", 5)
    self.declare_parameter("~err_grenze_da", 7)
    self.h_min = self.get_parameter("~h_min").get_parameter_value().integer_value
    self.h_max = self.get_parameter("~h_max").get_parameter_value().integer_value
    self.s_min = self.get_parameter("~s_min").get_parameter_value().integer_value
    self.s_max = self.get_parameter("~s_max").get_parameter_value().integer_value
    self.v_min = self.get_parameter("~v_min").get_parameter_value().integer_value
    self.v_max = self.get_parameter("~v_max").get_parameter_value().integer_value
    self.run = self.get_parameter("~run").get_parameter_value().integer_value
    self.x_speed_10 = self.get_parameter("~x_speed_10").get_parameter_value().integer_value
    self.err_grenze = self.get_parameter("~err_grenze").get_parameter_value().integer_value
    self.z_speed = self.get_parameter("~z_speed").get_parameter_value().integer_value
    self.zwx_10 = self.get_parameter("~zwx_10").get_parameter_value().integer_value
    self.err_grenze_da = self.get_parameter("~err_grenze_da").get_parameter_value().integer_value
    cv2.namedWindow("Parameters")
    #cv2.resizeWindow("Parameters", 640, 320);
    cv2.moveWindow("Parameters",20,20)
    self.test = cv2.createTrackbar("h_min", "Parameters", self.h_min, 255, self.set_h_min)
    cv2.createTrackbar("h_max", "Parameters", self.h_max, 255, self.set_h_max)
    cv2.createTrackbar("s_min", "Parameters", self.s_min, 255, self.set_s_min)
    cv2.createTrackbar("s_max", "Parameters", self.s_max, 255, self.set_s_max)
    cv2.createTrackbar("v_min", "Parameters", self.v_min, 255, self.set_v_min)
    cv2.createTrackbar("v_max", "Parameters", self.v_max, 255, self.set_v_max)
    cv2.createTrackbar("run", "Parameters", self.run, 1, self.set_run)
    cv2.createTrackbar("x_speed_10", "Parameters", self.x_speed_10, 10, self.set_x_speed_10)
    cv2.createTrackbar("err_grenze", "Parameters", self.err_grenze, 10, self.set_err_grenze)
    cv2.createTrackbar("z_speed", "Parameters", self.z_speed , 200, self.set_z_speed)
    cv2.createTrackbar("zwx_10", "Parameters", self.zwx_10 , 10, self.set_zwx_10)
    cv2.createTrackbar("err_grenze_da", "Parameters", self.err_grenze_da , 10, self.set_err_grenze_da)
    self.control_run = self.create_subscription(Float64,'/control/max_vel',self.con_run,QoSProfile(depth=1))
    self.control_detect_level_run = self.create_subscription(Float64,'/control/detect_level/max_vel',self.con_detect_level_run,QoSProfile(depth=1))
    self.nofindcounter=0
    self.maxmax = 0.12
    self.status_i=0
    self.status_s=[1 ,2 ,2 ,1]
  def con_run(self, msg):
    self.get_logger().info("12345678%lf" %(self.maxmax))
    self.maxmax = msg.data #速度 
    self.get_logger().info("45567!%lf" %(self.maxmax))
  def con_detect_level_run(self, msg):
    self.get_logger().info("detectlevel_vel:%lf" % (self.max_detect_level))
    self.max_detect_level = msg.data

  def set_h_min(self, pos):
    self.h_min = pos
  def set_h_max(self, pos):
    self.h_max = pos
  def set_s_min(self, pos):
    self.s_min = pos
  def set_s_max(self, pos):
    self.s_max = pos
  def set_v_min(self, pos):
    self.v_min = pos
  def set_v_max(self, pos):
    self.v_max = pos
  def set_run(self, pos):
    self.run = pos
  def set_x_speed_10(self, pos):
    self.x_speed_10 = pos
  def set_err_grenze(self, pos):
    self.err_grenze = pos
  def set_z_speed(self, pos):
    self.z_speed = pos
  def set_zwx_10(self, pos):
     self.zwx_10=pos
  def set_err_grenze_da(self, pos):
    self.err_grenze_da=pos
  def image_callback(self, msg):
    global bridge
    np_arr = np.frombuffer(msg.data,np.uint8) 
    self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    image = np.copy(self.cv_image)
    #image = bridge.imgmsg_to_cv2(msg, 'bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    self.colorLower = (self.h_min, self.s_min, self.v_min)
    self.colorUpper = (self.h_max, self.s_max, self.v_max)
    mask = cv2.inRange(hsv, self.colorLower, self.colorUpper)
    h, w, d = image.shape
    search_top = 9 * h // 10
    search_bot = h
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)
      # BEGIN CONTROL
      err = cx - w / 2
      # print(err)
      self.get_logger().info("旋转角度：%s" % (err))
      # print(M)
      self.twist.linear.x = float(self.x_speed_10)/10
      # self.get_logger().info("!!!!!!!!!!!!!!!!!!!!!!!%lf" %(self.maxmax))
      if(self.maxmax > 0.05 and self.max_detect_level> 0.05):
        if(abs(err)>self.err_grenze):
          self.twist.angular.z = -float(err) / self.z_speed
        if(abs(err)>self.err_grenze_da):
         self.twist.linear.x = float(self.zwx_10)/10
      else: 
         self.twist.angular.z = 0.0
         self.twist.linear.x = 0.0
      self.nofindcounter = 0
      self.status_i=0
        #   self.angleBuffer.append(err)
        # if len(self.angleBuffer) > 2:
     #    self.twist.angular.z = -float(self.angleBuffer[0]) / 100
     #    self.angleBuffer.pop(0)
     #  else:
      #   self.twist.angular.z = -float(0) /100
      if self.run == 1:
        self.cmd_vel_pub.publish(self.twist)
      # END CONTROL
    else:
      self.get_logger().info("没有发现线存在,请调节hsv值")
      self.nofindcounter+=1
      if self.nofindcounter % 60 == 0 and self.status_s[self.status_i] == 2:
         self.status_i = (self.status_i+1)%4
         self.get_logger().info("@@@z+1.0")
         self.twist.angular.z = 2.0
         self.twist.linear.x = 0.0
         if self.run == 1:
           self.cmd_vel_pub.publish(self.twist)
         
      elif self.nofindcounter %60 == 0 and self.status_s[self.status_i] == 1:
         self.status_i = (self.status_i+1)%4
         self.get_logger().info("@@@z-1.0")
         self.twist.angular.z = -2.0
         self.twist.linear.x = 0.0
         if self.run == 1:
           self.cmd_vel_pub.publish(self.twist)
         
         
      
      
    cv2.resize(mask,(640,480))
    #cv2.imshow("mask", cv2.resize(mask,(500,500),interpolation=cv2.INTER_CUBIC))
    #cv2.resizeWindow("mask", 500, 500)
    cv2.resize(image,(640,480))
    #cv2.imshow("output", cv2.resize(image,(500,500),interpolation=cv2.INTER_CUBIC))
    #cv2.resizeWindow("output", 500, 500)
    #print('mask shape:', mask.shape)
    #print('image shape:', image.shape)
    outputs = self.stackImages(0.8,([image,mask]))
    cv2.imshow("Parameters", outputs)
    cv2.waitKey(3)

  def stackImages(self,scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver



def main(args=None):
  rclpy.init(args=args)

  line_follower_node = LineFollower()

  rclpy.spin(line_follower_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  line_follower_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

