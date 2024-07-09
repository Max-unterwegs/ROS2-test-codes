#!/usr/bin/env python3
import rclpy, cv2, numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import math
bridge = CvBridge()  # 转换为ros2的消息类型(imgmsg)的工具
import numpy as np
from rclpy.qos import QoSProfile
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import CompressedImage
from enum import Enum

class LineFollower(Node):
  def __init__(self):
    super().__init__('line_follower')
    self.image_sub = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
    #self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
    self.image_sub
    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 5)
    self.twist = Twist()
    self.angleBuffer = []
    self.colorLower = None
    self.colorUpper = None
    self.declare_parameter("~h_min", 25)
    self.declare_parameter("~h_max", 55)
    self.declare_parameter("~s_min", 40)
    self.declare_parameter("~s_max", 255)
    self.declare_parameter("~v_min", 40)
    self.declare_parameter("~v_max", 255)
    self.declare_parameter("~run", 0)
    self.h_min = self.get_parameter("~h_min").get_parameter_value().integer_value
    self.h_max = self.get_parameter("~h_max").get_parameter_value().integer_value
    self.s_min = self.get_parameter("~s_min").get_parameter_value().integer_value
    self.s_max = self.get_parameter("~s_max").get_parameter_value().integer_value
    self.v_min = self.get_parameter("~v_min").get_parameter_value().integer_value
    self.v_max = self.get_parameter("~v_max").get_parameter_value().integer_value
    self.run = self.get_parameter("~run").get_parameter_value().integer_value
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
    self.declare_parameter("~detect/lane/red/hue_l", 0)
    self.declare_parameter("~detect/lane/red/hue_h", 255)
    self.declare_parameter("~detect/lane/red/saturation_l",128)
    self.declare_parameter("~detect/lane/red/saturation_h", 255)
    self.declare_parameter("~detect/lane/red/lightness_l", 48)
    self.declare_parameter("~detect/lane/red/lightness_h", 255)
    self.hue_red_l = self.get_parameter('~detect/lane/red/hue_l').get_parameter_value().integer_value
    self.hue_red_h = self.get_parameter('~detect/lane/red/hue_h').get_parameter_value().integer_value
    self.saturation_red_l = self.get_parameter('~detect/lane/red/saturation_l').get_parameter_value().integer_value
    self.saturation_red_h = self.get_parameter('~detect/lane/red/saturation_h').get_parameter_value().integer_value
    self.lightness_red_l = self.get_parameter('~detect/lane/red/lightness_l').get_parameter_value().integer_value
    self.lightness_red_h = self.get_parameter('~detect/lane/red/lightness_h').get_parameter_value().integer_value    
    self.declare_parameter("~detect/lane/yellow/hue_l", 40)
    self.declare_parameter("~detect/lane/yellow/hue_h", 40)
    self.declare_parameter("~detect/lane/yellow/saturation_l", 100)
    self.declare_parameter("~detect/lane/yellow/saturation_h", 255)
    self.declare_parameter("~detect/lane/yellow/lightness_l", 100)
    self.declare_parameter("~detect/lane/yellow/lightness_h", 255)
    self.hue_yellow_l = self.get_parameter('~detect/lane/yellow/hue_l').get_parameter_value().integer_value
    self.hue_yellow_h = self.get_parameter('~detect/lane/yellow/hue_h').get_parameter_value().integer_value
    self.saturation_yellow_l = self.get_parameter('~detect/lane/yellow/saturation_l').get_parameter_value().integer_value
    self.saturation_yellow_h = self.get_parameter('~detect/lane/yellow/saturation_h').get_parameter_value().integer_value
    self.lightness_yellow_l = self.get_parameter('~detect/lane/yellow/lightness_l').get_parameter_value().integer_value
    self.lightness_yellow_h = self.get_parameter('~detect/lane/yellow/lightness_h').get_parameter_value().integer_value
    self.declare_parameter("~detect/lane/green/hue_l", 35)
    self.declare_parameter("~detect/lane/green/hue_h", 105)
    self.declare_parameter("~detect/lane/green/saturation_l", 102)
    self.declare_parameter("~detect/lane/green/saturation_h", 255)
    self.declare_parameter("~detect/lane/green/lightness_l", 102)
    self.declare_parameter("~detect/lane/green/lightness_h", 255)
    self.hue_green_l = self.get_parameter('~detect/lane/green/hue_l').get_parameter_value().integer_value
    self.hue_green_h = self.get_parameter('~detect/lane/green/hue_h').get_parameter_value().integer_value
    self.saturation_green_l = self.get_parameter('~detect/lane/green/saturation_l').get_parameter_value().integer_value
    self.saturation_green_h = self.get_parameter('~detect/lane/green/saturation_h').get_parameter_value().integer_value
    self.lightness_green_l = self.get_parameter('~detect/lane/green/lightness_l').get_parameter_value().integer_value
    self.lightness_green_h = self.get_parameter('~detect/lane/green/lightness_h').get_parameter_value().integer_value
    self.declare_parameter("~is_detection_calibration_mode", True)
    self.is_calibration_mode = self.get_parameter("~is_detection_calibration_mode").get_parameter_value().bool_value
        
        #if self.is_calibration_mode == True:
        #    srv_detect_lane = Server(DetectTrafficLightParamsConfig, self.cbGetDetectTrafficLightParam)

    self.sub_image_type = "compressed"          # "compressed" / "raw"
    self.pub_image_type = "compressed"          # "compressed" / "raw"

    self.counter = 1
        
    if self.sub_image_type == "compressed":
            # subscribes compressed image
        self.sub_image_original = self.create_subscription(CompressedImage, '/oakd/rgb/preview/image_raw/compressed',  self.cbGetImage, QoSProfile(depth=1))
    elif self.sub_image_type == "raw":
            # subscribes raw image
        self.sub_image_original = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.cbGetImage, QoSProfile(depth=1))
 
    if self.pub_image_type == "compressed":
            # publishes compensated image in compressed type 
        self.pub_image_traffic_light = self.create_publisher(CompressedImage, '/detect/image_output/compressed',  QoSProfile(depth=1))
    elif self.pub_image_type == "raw":
            # publishes compensated image in raw type
        self.pub_image_traffic_light = self.create_publisher(Image, '/detect/image_output', QoSProfile(depth=1))

    if self.is_calibration_mode == True:
        if self.pub_image_type == "compressed":
                # publishes light image in compressed type 
            self.pub_image_red_light = self.create_publisher(CompressedImage, '/detect/image_output_sub1/compressed', QoSProfile(depth=1))
            self.pub_image_yellow_light = self.create_publisher(CompressedImage, '/detect/image_output_sub2/compressed', QoSProfile(depth=1))
            self.pub_image_green_light = self.create_publisher(CompressedImage, '/detect/image_output_sub3/compressed', QoSProfile(depth=1))
        elif self.pub_image_type == "raw":
                # publishes light image in raw type
            self.pub_image_red_light = self.create_publisher(Image, '/detect/image_output_sub1', QoSProfile(depth=1))
            self.pub_image_yellow_light = self.create_publisher(Image, '/detect/image_output_sub2', QoSProfile(depth=1))
            self.pub_image_green_light = self.create_publisher(Image, '/detect/image_output_sub3', QoSProfile(depth=1))

    self.sub_traffic_light_finished = self.create_subscription(UInt8, '/control/traffic_light_finished',  self.cbTrafficLightFinished, QoSProfile(depth=1))
    self.pub_traffic_light_return = self.create_publisher(UInt8, '/detect/traffic_light_stamped', QoSProfile(depth=1))
    self.pub_parking_start = self.create_publisher(UInt8, '/control/traffic_light_start', QoSProfile(depth=1))
    self.pub_max_vel = self.create_publisher(Float64, '/control/max_vel', QoSProfile(depth=1))

    self.StepOfTrafficLight = Enum('StepOfTrafficLight', 'searching_traffic_light searching_green_light searching_yellow_light searching_red_light waiting_green_light pass_traffic_light')

    self.cvBridge = CvBridge()
    self.cv_image = None

    self.is_image_available = False
    self.is_traffic_light_finished = False

    self.green_count = 0
    self.yellow_count = 0
    self.red_count = 0
    self.stop_count = 0
    self.off_traffic = False
        #rospy.sleep(1)

        #loop_rate = rospy.Rate(10)
        #while not rospy.is_shutdown():
        #    if self.is_image_available == True:
        #        self.fnFindTrafficLight()

        #    loop_rate.sleep()

  def cbGetDetectTrafficLightParam(self, config, level):
        rospy.loginfo("[Detect Traffic Light] Detect Traffic Light Calibration Parameter reconfigured to")
        rospy.loginfo("hue_red_l : %d", config.hue_red_l)
        rospy.loginfo("hue_red_h : %d", config.hue_red_h)
        rospy.loginfo("saturation_red_l : %d", config.saturation_red_l)
        rospy.loginfo("saturation_red_h : %d", config.saturation_red_h)
        rospy.loginfo("lightness_red_l : %d", config.lightness_red_l)
        rospy.loginfo("lightness_red_h : %d", config.lightness_red_h)

        rospy.loginfo("hue_yellow_l : %d", config.hue_yellow_l)
        rospy.loginfo("hue_yellow_h : %d", config.hue_yellow_h)
        rospy.loginfo("saturation_yellow_l : %d", config.saturation_yellow_l)
        rospy.loginfo("saturation_yellow_h : %d", config.saturation_yellow_h)
        rospy.loginfo("lightness_yellow_l : %d", config.lightness_yellow_l)
        rospy.loginfo("lightness_yellow_h : %d", config.lightness_yellow_h)

        rospy.loginfo("hue_green_l : %d", config.hue_green_l)
        rospy.loginfo("hue_green_h : %d", config.hue_green_h)
        rospy.loginfo("saturation_green_l : %d", config.saturation_green_l)
        rospy.loginfo("saturation_green_h : %d", config.saturation_green_h)
        rospy.loginfo("lightness_green_l : %d", config.lightness_green_l)
        rospy.loginfo("lightness_green_h : %d", config.lightness_green_h)

        self.hue_red_l = config.hue_red_l
        self.hue_red_h = config.hue_red_h
        self.saturation_red_l = config.saturation_red_l
        self.saturation_red_h = config.saturation_red_h
        self.lightness_red_l = config.lightness_red_l
        self.lightness_red_h = config.lightness_red_h

        self.hue_yellow_l = config.hue_yellow_l
        self.hue_yellow_h = config.hue_yellow_h
        self.saturation_yellow_l = config.saturation_yellow_l
        self.saturation_yellow_h = config.saturation_yellow_h
        self.lightness_yellow_l = config.lightness_yellow_l
        self.lightness_yellow_h = config.lightness_yellow_h

        self.hue_green_l = config.hue_green_l
        self.hue_green_h = config.hue_green_h
        self.saturation_green_l = config.saturation_green_l
        self.saturation_green_h = config.saturation_green_h
        self.lightness_green_l = config.lightness_green_l
        self.lightness_green_h = config.lightness_green_h

        return config


  def cbGetImage(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 5 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == "compressed":
            #np_arr = np.fromstring(image_msg.data, np.uint8)
            np_arr = np.frombuffer(image_msg.data,np.uint8) 
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        self.fnFindTrafficLight()
  def fnFindTrafficLight(self):
        cv_image_mask = self.fnMaskGreenTrafficLight()
        cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)

        status1 = self.fnFindCircleOfTrafficLight(cv_image_mask, 'green')
        self.get_logger().info(str(status1))
        if status1 == 1 or status1 == 5:
            self.stop_count = 0
            self.red_count = 0
            self.green_count += 1
        else:
            self.green_count = 0
        
            cv_image_mask = self.fnMaskYellowTrafficLight()
            cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)

            status2 = self.fnFindCircleOfTrafficLight(cv_image_mask, 'yellow')
            if status2 == 2:
                self.yellow_count += 1
            else:
                self.yellow_count = 0

                cv_image_mask = self.fnMaskRedTrafficLight()
                cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)

                status3 = self.fnFindCircleOfTrafficLight(cv_image_mask, 'red')
                self.get_logger().info(str(status3))
                if status3 == 3:
                    self.red_count += 1
                    self.stop_count += 1
                    self.green_count = 0
                elif status3 == 4:
                    self.red_count = 0
                    self.stop_count += 1
                else:
                    self.red_count = 0
                    self.stop_count = 0

        if self.green_count > 3:
            self.get_logger().info("11111111")
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.12
            self.pub_max_vel.publish(msg_pub_max_vel)
            self.get_logger().info("GREEN")
            start = UInt8()
            start.data = 1
            self.run=1
            self.pub_parking_start.publish(start)
            cv2.putText(self.cv_image,"GREEN", (self.point_col, self.point_low), cv2.FONT_HERSHEY_DUPLEX, 0.5, (80, 255, 0))

        #if self.yellow_count > 12:
            #msg_pub_max_vel = Float64()
            #msg_pub_max_vel.data = 0.06 if not self.off_traffic else 0.12
            #self.pub_max_vel.publish(msg_pub_max_vel)
            #self.get_logger().info("YELLOW")
            #cv2.putText(self.cv_image,"YELLOW", (self.point_col, self.point_low), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 255))

        if self.red_count > 3:
            #self.red_count = 0
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.03
            self.pub_max_vel.publish(msg_pub_max_vel)
            self.run=0
            self.get_logger().info("RED")
            cv2.putText(self.cv_image,"RED", (self.point_col, self.point_low), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255))

        if self.stop_count > 8:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.0
            self.pub_max_vel.publish(msg_pub_max_vel)  
            self.get_logger().info("STOP")
            self.off_traffic = True
            start = UInt8()
            start.data = 0
            self.pub_parking_start.publish(start)
            cv2.putText(self.cv_image,"STOP", (self.point_col, self.point_low), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255))

        if self.pub_image_type == "compressed":
            # publishes traffic light image in compressed type
            self.pub_image_traffic_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(self.cv_image, "jpg"))

        elif self.pub_image_type == "raw":
            # publishes traffic light image in raw type
            self.pub_image_traffic_light.publish(self.cvBridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
  def fnMaskRedTrafficLight(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_red_l
        Hue_h = self.hue_red_h
        Saturation_l = self.saturation_red_l
        Saturation_h = self.saturation_red_h
        Lightness_l = self.lightness_red_l
        Lightness_h = self.lightness_red_h

        # define range of red color in HSV
        lower_red = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_red = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes red light filtered image in compressed type
                self.pub_image_red_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes red light filtered image in raw type
                self.pub_image_red_light.publish(self.cvBridge.cv2_to_imgmsg(mask, "mono8"))

        mask = cv2.bitwise_not(mask)

        return mask
  def fnMaskYellowTrafficLight(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_yellow_l
        Hue_h = self.hue_yellow_h
        Saturation_l = self.saturation_yellow_l
        Saturation_h = self.saturation_yellow_h
        Lightness_l = self.lightness_yellow_l
        Lightness_h = self.lightness_yellow_h

        # define range of yellow color in HSV
        lower_yellow = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes yellow light filtered image in compressed type
                self.pub_image_yellow_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes yellow light filtered image in raw type
                self.pub_image_yellow_light.publish(self.cvBridge.cv2_to_imgmsg(mask, "mono8"))

        mask = cv2.bitwise_not(mask)

        return mask
  def fnMaskGreenTrafficLight(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_green_l
        Hue_h = self.hue_green_h
        Saturation_l = self.saturation_green_l
        Saturation_h = self.saturation_green_h
        Lightness_l = self.lightness_green_l
        Lightness_h = self.lightness_green_h

        # define range of green color in HSV
        lower_green = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_green = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes green light filtered image in compressed type
                self.pub_image_green_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes green light filtered image in raw type
                self.pub_image_green_light.publish(self.cvBridge.cv2_to_imgmsg(mask, "mono8"))

        mask = cv2.bitwise_not(mask)

        return mask
  def fnFindCircleOfTrafficLight(self, mask, find_color):
        status = 0

        params=cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 50
        params.maxArea = 600

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.6

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.6

        det=cv2.SimpleBlobDetector_create(params)
        keypts=det.detect(mask)
        frame=cv2.drawKeypoints(self.cv_image,keypts,np.array([]),(0,255,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        col1 = 180
        col2 = 270
        col3 = 305

        low1 = 50
        low2 = 170
        low3 = 170
        
        # if detected more than 1 light
        for i in range(len(keypts)):
            self.point_col = int(keypts[i].pt[0])
            self.point_low = int(keypts[i].pt[1])
            #print(self.point_col)
            #print(self.point_low)
            if True:#self.point_col > col1 and self.point_col < col2 and self.point_low > low1 and self.point_low < low2:
                print(find_color)
                if find_color == 'green':
                    status = 1
            #    elif find_color == 'yellow':
             #       status = 2
                elif find_color == 'red':
                    status = 3
           # elif self.point_col > col2 and self.point_col < col3 and self.point_low > low1 and self.point_low < low3:
              #  if find_color == 'red':
              #      status = 4
              #  elif find_color == 'green':
              #      status = 5
            else:
                status = 6

        return status
  def cbTrafficLightFinished(self, traffic_light_finished_msg):
        self.is_traffic_light_finished = True


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

  def image_callback(self, msg):
    global bridge
    image = bridge.imgmsg_to_cv2(msg, 'bgr8')
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
      self.twist.linear.x = 0.2
      if(abs(err)>10):
        self.twist.angular.z = -float(err) / 100
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
