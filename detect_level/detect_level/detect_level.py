import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange, SetParametersResult
import math
from enum import Enum
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import time
#from dynamic_reconfigure.server import Server
#from turtlebot3_autorace_detect.cfg import DetectLevelParamsConfig

def fnCalcDistanceDot2Line(a, b, c, x0, y0):
    distance = abs(x0*a + y0*b + c)/math.sqrt(a*a + b*b)
    return distance

def fnCalcDistanceDot2Dot(x1, y1, x2, y2):
    distance = math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
    return distance

def fnArrangeIndexOfPoint(arr):
    new_arr = arr[:]
    arr_idx = [0] * len(arr)
    for i in range(len(arr)):
        arr_idx[i] = i

    for i in range(len(arr)):
        for j in range(i+1, len(arr)):
            if arr[i] < arr[j]:
                buffer = arr_idx[j]
                arr_idx[j] = arr_idx[i]
                arr_idx[i] = buffer
                buffer = new_arr[j]
                new_arr[j] = new_arr[i]
                new_arr[i] = buffer
    return arr_idx

def fnCheckLinearity(point1, point2, point3):
    threshold_linearity = 50
    x1, y1 = point1
    x2, y2 = point3
    if x2-x1 != 0:
        a = (y2-y1)/(x2-x1)
    else:
        a = 1000
    b = -1
    c = y1 - a*x1
    err = fnCalcDistanceDot2Line(a, b, c, point2[0], point2[1])

    if err < threshold_linearity:
        return True
    else:
        return False

def fnCheckDistanceIsEqual(point1, point2, point3):

    threshold_distance_equality = 3
    distance1 = fnCalcDistanceDot2Dot(point1[0], point1[1], point2[0], point2[1])
    distance2 = fnCalcDistanceDot2Dot(point2[0], point2[1], point3[0], point3[1])
    std = np.std([distance1, distance2])

    if std < threshold_distance_equality:
        return True
    else:
        return False

class DetectLevel(Node):
    def __init__(self):
        super().__init__('detect_level')
        hsv_parameter_descriptor = ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='An integer parameter that can be adjusted using a slider.',
                integer_range=[
                    IntegerRange(from_value=0, to_value=255, step=1),
                ]
            )
        self.declare_parameter("~detect/level/red/hue_l", 0, hsv_parameter_descriptor)
        self.declare_parameter("~detect/level/red/hue_h", 255, hsv_parameter_descriptor)
        self.declare_parameter("~detect/level/red/saturation_l", 128, hsv_parameter_descriptor)
        self.declare_parameter("~detect/level/red/saturation_h", 253, hsv_parameter_descriptor)
        self.declare_parameter("~detect/level/red/lightness_l", 68, hsv_parameter_descriptor)
        self.declare_parameter("~detect/level/red/lightness_h", 255, hsv_parameter_descriptor)
        self.declare_parameter("~is_detection_calibration_mode", True)
        self.hue_red_l = self.get_parameter('~detect/level/red/hue_l').get_parameter_value().integer_value
        self.hue_red_h = self.get_parameter('~detect/level/red/hue_h').get_parameter_value().integer_value
        self.saturation_red_l = self.get_parameter('~detect/level/red/saturation_l').get_parameter_value().integer_value
        self.saturation_red_h = self.get_parameter('~detect/level/red/saturation_h').get_parameter_value().integer_value
        self.lightness_red_l = self.get_parameter('~detect/level/red/lightness_l').get_parameter_value().integer_value
        self.lightness_red_h = self.get_parameter('~detect/level/red/lightness_h').get_parameter_value().integer_value

        self.is_calibration_mode = self.get_parameter("~is_detection_calibration_mode").get_parameter_value().bool_value
        # //? server?? what is this?
        #if self.is_calibration_mode == True:
        #    srv_detect_lane = Server(DetectLevelParamsConfig, self.cbGetDetectLevelParam)

        self.sub_image_type = "compressed" # you can choose image type "compressed", "raw"
        self.pub_image_type = "compressed" # you can choose image type "compressed", "raw"
        
        # //*GET the image of the camera
        if self.sub_image_type == "compressed":
            # subscribes compressed image
            self.sub_image_original = self.create_subscription(CompressedImage, '/oakd/rgb/preview/image_raw/compressed', self.cbGetImage, QoSProfile(depth=1))
            self.sub_image_original
        elif self.sub_image_type == "raw":
            # subscribes raw image
            self.sub_image_original = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.cbGetImage, QoSProfile(depth=1))
            self.sub_image_original
        
        if self.pub_image_type == "compressed":
            # publishes level image in compressed type 
            self.pub_image_level = self.create_publisher(CompressedImage, '/detect_level/image_output/compressed', QoSProfile(depth=1))
        elif self.pub_image_type == "raw":
            # publishes level image in raw type
            self.pub_image_level = self.create_publisher(Image, '/detect_level/image_output', QoSProfile(depth=1))
        
        # //? what is this
        if self.is_calibration_mode == True:
            
            if self.pub_image_type == "compressed":
                # publishes color filtered image in compressed type 
                 self.pub_image_color_filtered = self.create_publisher(CompressedImage, '/detect_level/image_output_sub1/compressed',  1)
            elif self.pub_image_type == "raw":
                # publishes color filtered image in raw type
                self.pub_image_color_filtered = self.create_publisher(Image, '/detect_level/image_output_sub1',  1)

        # //*GET the message of the state of machine
        self.sub_level_crossing_order = self.create_subscription(UInt8, '/detect/level_crossing_order',  self.cbLevelCrossingOrder, 1) # //*get the message of the state of machine to132
        self.sub_level_crossing_finished = self.create_subscription(UInt8, '/control/level_crossing_finished',  self.cbLevelCrossingFinished, 1) # //*get the message of the end of level crossing

        self.pub_level_crossing_return = self.create_publisher(UInt8, '/detect/level_crossing_stamped', 1) # //? WHAT IS THIS?
        self.pub_parking_start = self.create_publisher(UInt8, '/control/level_crossing_start', 1) # //? start?? see next

        self.pub_max_vel = self.create_publisher(Float64, '/control/detect_level/max_vel', 1)

        self.StepOfLevelCrossing = Enum('StepOfLevelCrossing', 'searching_stop_sign searching_level watching_level stop pass_level') # //* 默认从1开始

        self.cvBridge = CvBridge()
        self.cv_image = None

        self.stop_bar_count = 0

        self.is_level_crossing_finished = False

        self.counter = 1
        cv2.namedWindow('level_crossing')
        cv2.moveWindow("level_crossing",1080,20)
        cv2.createTrackbar('hue_red_l', 'level_crossing', self.hue_red_l, 255, self.set_h_min)
        cv2.createTrackbar('hue_red_h', 'level_crossing', self.hue_red_h, 255, self.set_h_max)
        cv2.createTrackbar('saturation_red_l', 'level_crossing', self.saturation_red_l, 255, self.set_s_min)
        cv2.createTrackbar('saturation_red_h', 'level_crossing', self.saturation_red_h, 255, self.set_s_max)
        cv2.createTrackbar('lightness_red_l', 'level_crossing', self.lightness_red_l, 255, self.set_v_min)
        cv2.createTrackbar('lightness_red_h', 'level_crossing', self.lightness_red_h, 255, self.set_v_max)
        
        
        #self.create_timer(1.0 / 15, self.fnFindLevel)
    def set_h_min(self, pos):
        self.hue_red_l = pos
    def set_h_max(self, pos):
        self.hue_red_h = pos
    def set_s_min(self, pos):
        self.saturation_red_l = pos
    def set_s_max(self, pos):
        self.saturation_red_h = pos
    def set_v_min(self, pos):
        self.lightness_red_l = pos
    def set_v_max(self, pos):
        self.lightness_red_h = pos

    
    def cbGetDetectLevelParam(self, config, level):
        self.get_logger().info("[Detect Level] Detect Level Calibration Parameter reconfigured to")
        self.get_logger().info("hue_red_l : %d" % config.hue_red_l)
        self.get_logger().info("hue_red_h : %d" % config.hue_red_h)
        self.get_logger().info("saturation_red_l : %d" % config.saturation_red_l)
        self.get_logger().info("saturation_red_h : %d" % config.saturation_red_h)
        self.get_logger().info("lightness_red_l : %d" % config.lightness_red_l)
        self.get_logger().info("lightness_red_h : %d" % config.lightness_red_h)
        
        self.hue_red_l = config.hue_red_l
        self.hue_red_h = config.hue_red_h
        self.saturation_red_l = config.saturation_red_l
        self.saturation_red_h = config.saturation_red_h
        self.lightness_red_l = config.lightness_red_l
        self.lightness_red_h = config.lightness_red_h

        return config

    def cbGetImage(self, image_msg):
        # //*Change the frame rate by yourself. Now, it is set to 1/3 (10fps). 
        # Unappropriate value of frame rate may cause huge delay on entire recognition process.
        # This is up to your computer's operating power.
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
        self.fnFindLevel()

   
    # @brief GET the order and do something
    # //* @param order : the order of the state of machine
    # //! @return void
    # //TODO : implement the function of each state
    # @param order : order    
    def cbLevelCrossingOrder(self, order):
        pub_level_crossing_return = UInt8()

        # if order.data == self.StepOfLevelCrossing.searching_stop_sign.value:
        #    # rospy.loginfo("Now lane_following")
        #    self.get_logger().info("Now searching_stop_sign")

        #    pub_level_crossing_return.data = self.StepOfLevelCrossing.searching_stop_sign.value
                                                            
        # elif order.data == self.StepOfLevelCrossing.searching_level.value:
        #     # rospy.loginfo("Now searching_level")
        #     self.get_logger().info("Now searching_level")

        #     # //* Time to slow down
        #     msg_pub_max_vel = Float64()
        #     msg_pub_max_vel.data = 0.10
        #     self.pub_max_vel.publish(msg_pub_max_vel)

        #     while True:
        #         is_level_detected, _, _ = self.fnFindLevel()
        #         if is_level_detected == True:
        #             break
        #         else:
        #             pass
            
        #   #  rospy.loginfo("SLOWDOWN!!")

        #     msg_pub_max_vel.data = 0.04
        #     self.pub_max_vel.publish(msg_pub_max_vel)

        #     pub_level_crossing_return.data = self.StepOfLevelCrossing.searching_level.value

        # elif order.data == self.StepOfLevelCrossing.watching_level.value:
        #    # rospy.loginfo("Now watching_level")

        #     while True:
        #         _, is_level_close, _ = self.fnFindLevel()
        #         if is_level_close == True:
        #             break
        #         else:
        #             pass

        #    # rospy.loginfo("STOP~~")

        #     msg_pub_max_vel = Float64()
        #     msg_pub_max_vel.data = 0.0
        #     self.pub_max_vel.publish(msg_pub_max_vel)

        #     pub_level_crossing_return.data = self.StepOfLevelCrossing.watching_level.value

        # elif order.data == self.StepOfLevelCrossing.stop.value:
        #   #  rospy.loginfo("Now stop")

        #     while True:
        #         _, _, is_level_opened = self.fnFindLevel()
        #         if is_level_opened == True:
        #             break
        #         else:
        #             pass

        #     # rospy.loginfo("GO~~")

        #     msg_pub_max_vel = Float64()
        #     msg_pub_max_vel.data = 0.04
        #     self.pub_max_vel.publish(msg_pub_max_vel)

        #     pub_level_crossing_return.data = self.StepOfLevelCrossing.stop.value

        # elif order.data == self.StepOfLevelCrossing.pass_level.value:
        #     # rospy.loginfo("Now pass_level")

        #     pub_level_crossing_return.data = self.StepOfLevelCrossing.pass_level.value

        # self.pub_level_crossing_return.publish(pub_level_crossing_return)

    def fnFindLevel(self):
        cv_image_mask = self.fnMaskRedOfLevel()

        cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)
        # cv2.imshow("detect", cv_image_mask)
        # cv2.imshow("image",self.cv_image)
        # cv2.waitKey(1)
        return self.fnFindRectOfLevel(cv_image_mask)

    def fnMaskRedOfLevel(self):
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

        # ## TODO: expand range for red
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # # Convert BGR to HSV
        # Hue_l = 0
        # Hue_h = 23
        # Saturation_l = 95
        # Saturation_h = 217
        # Lightness_l = 58
        # Lightness_h = 255

        # # define range of red color in HSV
        # lower_red = np.array([Hue_l, Saturation_l, Lightness_l])
        # upper_red = np.array([Hue_h, Saturation_h, Lightness_h])

        # # Threshold the HSV image to get only red colors
        # mask2 = cv2.inRange(hsv, lower_red, upper_red)

        # # Bitwise-AND mask and original image
        # # res2 = cv2.bitwise_and(image, image, mask = mask2)

        # mask = cv2.bitwise_or(mask, mask2, mask = None)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes yellow lane filtered image in compressed type
                self.pub_image_color_filtered.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes yellow lane filtered image in raw type
                self.pub_image_color_filtered.publish(self.cvBridge.cv2_to_imgmsg(mask, "mono8"))            

        mask = cv2.bitwise_not(mask)

        return mask


    def fnFindRectOfLevel(self, mask):
        is_level_detected = False
        is_level_close = False
        is_level_opened = False

        # //*  斑点检测器
        params=cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 200
        params.maxArea = 3000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.5

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.9

        det=cv2.SimpleBlobDetector_create(params)
        keypts=det.detect(mask)
        frame=cv2.drawKeypoints(self.cv_image,keypts,np.array([]),(0,255,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        mean_x = 0.0
        mean_y = 0.0

        # if detected 3 red rectangular
        if len(keypts) >= 3:
            is_level_detected = True
            for i in range(3):
                mean_x = mean_x + keypts[i].pt[0]/3
                mean_y = mean_y + keypts[i].pt[1]/3
            arr_distances = [0]*3
            for i in range(3):
                arr_distances[i] = fnCalcDistanceDot2Dot(mean_x, mean_y, keypts[i].pt[0], keypts[i].pt[1])

            # finding thr farthest point from the center
            self.get_logger().info("!!!!!!!")
            idx1, idx2, idx3 = fnArrangeIndexOfPoint(arr_distances)
            frame = cv2.line(frame, (int(keypts[idx1].pt[0]), int(keypts[idx1].pt[1])), (int(keypts[idx2].pt[0]), int(keypts[idx2].pt[1])), (255, 0, 0), 5)
            frame = cv2.line(frame, (int(mean_x), int(mean_y)), (int(mean_x), int(mean_y)), (255, 255, 0), 5)
            point1 =  [int(keypts[idx1].pt[0]), int(keypts[idx1].pt[1]-1)]
            point2 = [int(keypts[idx3].pt[0]), int(keypts[idx3].pt[1]-1)]
            point3 = [int(keypts[idx2].pt[0]), int(keypts[idx2].pt[1]-1)]
            # 计算斜率和角度
            slope = (point2[1] - point1[1]) / (point2[0] - point1[0])
            angle = math.atan(slope) * 180 / math.pi
            # 比较角度和60度
            if abs(angle) < 60:
                is_level_close = True
                is_level_opened = False
                self.get_logger().info("angle:%f"% abs(angle))  
            else:
                is_level_close = False
                is_level_opened = True
                self.get_logger().info("angle:%f"% abs(angle))  

            # test linearity and distance equality. If satisfy the both, continue to next process
            is_rects_linear = fnCheckLinearity(point1, point2, point3)
            is_rects_dist_equal = fnCheckDistanceIsEqual(point1, point2, point3)
            
            # print("hello!")
            
            if is_rects_linear == True or is_rects_dist_equal == True:
                # finding the angle of line
                self.get_logger().info("is linear!!")
                distance_bar2car = 100 / fnCalcDistanceDot2Dot(point1[0], point1[1], point2[0], point2[1])
            else:
                self.get_logger().info("no detect!!")
                
                # publishing topic
                # self.stop_bar_count = 40
                # if distance_bar2car > 0.8:
                #     is_level_detected = True
                #     self.stop_bar_state = 'slowdown'
                #     self.state = "detected"
                # else:
                #     is_level_close = True
                #     self.stop_bar_state = 'stop'
        else:
            print("no 3 rect!")
            is_level_detected = False            
        # if self.stop_bar_count > 0:
        #     self.stop_bar_count -= 1
        # if self.stop_bar_count == 0:
        #     is_level_opened = True
        #     self.stop_bar_state = 'go'
        
        if self.pub_image_type == "compressed":
            # publishes level image in compressed type
            self.pub_image_level.publish(self.cvBridge.cv2_to_compressed_imgmsg(frame, "jpg"))

        elif self.pub_image_type == "raw":
            # publishes level image in raw type
            self.pub_image_level.publish(self.cvBridge.cv2_to_imgmsg(frame, "bgr8"))
        cv2.resize(mask,(640,480))
        cv2.resize(frame,(640,480))
        output = self.stackImages(0.8,([frame,mask]))
        cv2.imshow("level_crossing", output)
        cv2.waitKey(1)
        
        # //*begin moving    
        self.get_logger().info("is_level_detected: %s, is_level_close: %s, is_level_opened: %s" % (is_level_detected, is_level_close, is_level_opened))
        if is_level_detected == True:
            self.get_logger().info("Level Detected")
            if is_level_close == True:
                self.get_logger().info("Level Close")
                msg_pub_max_vel = Float64()
                msg_pub_max_vel.data = 0.04
                self.pub_max_vel.publish(msg_pub_max_vel)
            else:
                self.get_logger().info("Level Opened")
                msg_pub_max_vel = Float64()
                msg_pub_max_vel.data = 0.12
                time.sleep(3)
                self.pub_max_vel.publish(msg_pub_max_vel)
            
        else:
            self.get_logger().info("Level Not Detected")
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.12
            self.pub_max_vel.publish(msg_pub_max_vel)

        return is_level_detected, is_level_close, is_level_opened

    def cbLevelCrossingFinished(self, level_crossing_finished_msg):
        self.is_level_crossing_finished = True

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

    # def main(self):
        # rospy.spin()


def main(args=None):
    rclpy.init(args=args)
    level_detector = DetectLevel()
    rclpy.spin(level_detector)
    #level_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
