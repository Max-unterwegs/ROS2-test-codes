import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2
from enum import Enum
from cv_bridge import CvBridge, CvBridgeError
#from dynamic_reconfigure.server import Server
#from turtlebot3_autorace_detect.cfg import DetectTrafficLightParamsConfig

class DetectTrafficLight(Node):
    def __init__(self):
        super().__init__('detect_traffic_light')
        #self.get_logger().info('start detect traffic light node')
        self.declare_parameter("~detect/lane/red/hue_l", 139)
        self.declare_parameter("~detect/lane/red/hue_h", 255)
        self.declare_parameter("~detect/lane/red/saturation_l", 86)
        self.declare_parameter("~detect/lane/red/saturation_h", 255)
        self.declare_parameter("~detect/lane/red/lightness_l", 191)
        self.declare_parameter("~detect/lane/red/lightness_h", 255)
        self.hue_red_l = self.get_parameter('~detect/lane/red/hue_l').get_parameter_value().integer_value
        self.hue_red_h = self.get_parameter('~detect/lane/red/hue_h').get_parameter_value().integer_value
        self.saturation_red_l = self.get_parameter('~detect/lane/red/saturation_l').get_parameter_value().integer_value
        self.saturation_red_h = self.get_parameter('~detect/lane/red/saturation_h').get_parameter_value().integer_value
        self.lightness_red_l = self.get_parameter('~detect/lane/red/lightness_l').get_parameter_value().integer_value
        self.lightness_red_h = self.get_parameter('~detect/lane/red/lightness_h').get_parameter_value().integer_value
        
        self.declare_parameter("~detect/lane/yellow/hue_l", 20)
        self.declare_parameter("~detect/lane/yellow/hue_h", 35)
        self.declare_parameter("~detect/lane/yellow/saturation_l", 100)
        self.declare_parameter("~detect/lane/yellow/saturation_h", 255)
        self.declare_parameter("~detect/lane/yellow/lightness_l", 50)
        self.declare_parameter("~detect/lane/yellow/lightness_h", 255)
        self.hue_yellow_l = self.get_parameter('~detect/lane/yellow/hue_l').get_parameter_value().integer_value
        self.hue_yellow_h = self.get_parameter('~detect/lane/yellow/hue_h').get_parameter_value().integer_value
        self.saturation_yellow_l = self.get_parameter('~detect/lane/yellow/saturation_l').get_parameter_value().integer_value
        self.saturation_yellow_h = self.get_parameter('~detect/lane/yellow/saturation_h').get_parameter_value().integer_value
        self.lightness_yellow_l = self.get_parameter('~detect/lane/yellow/lightness_l').get_parameter_value().integer_value
        self.lightness_yellow_h = self.get_parameter('~detect/lane/yellow/lightness_h').get_parameter_value().integer_value
        
        self.declare_parameter("~detect/lane/green/hue_l", 43)
        self.declare_parameter("~detect/lane/green/hue_h", 132)
        self.declare_parameter("~detect/lane/green/saturation_l", 93)
        self.declare_parameter("~detect/lane/green/saturation_h", 255)
        self.declare_parameter("~detect/lane/green/lightness_l", 224)
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
        #self.get_logger().info('start create subscriber and publisher')
        if self.sub_image_type == "compressed":
            # subscribes compressed image
            self.sub_image_original = self.create_subscription(CompressedImage, '/oakd/rgb/preview/image_raw/compressed',  self.cbGetImage, QoSProfile(depth=1))
        elif self.sub_image_type == "raw":
            # subscribes raw image
            self.sub_image_original = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.cbGetImage, QoSProfile(depth=1))
        #self.get_logger().info('start create publisher image')
        if self.pub_image_type == "compressed":
            # publishes compensated image in compressed type 
            self.pub_image_traffic_light = self.create_publisher(CompressedImage, '/detect/image_output/compressed',  QoSProfile(depth=1))
        elif self.pub_image_type == "raw":
            # publishes compensated image in raw type
            self.pub_image_traffic_light = self.create_publisher(Image, '/detect/image_output', QoSProfile(depth=1))
        #self.get_logger().info('start create publisher light')
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
        #self.get_logger().info('start create subscriber and publisher 2')
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
        #self.get_logger().info('start create ende')
        #rospy.sleep(1)

        #loop_rate = rospy.Rate(10)
        #while not rospy.is_shutdown():
        #    if self.is_image_available == True:
        #        self.fnFindTrafficLight()

        #    loop_rate.sleep()

    def cbGetDetectTrafficLightParam(self, config, level):
        # rospy.loginfo("[Detect Traffic Light] Detect Traffic Light Calibration Parameter reconfigured to")
        # rospy.loginfo("hue_red_l : %d", config.hue_red_l)
        # rospy.loginfo("hue_red_h : %d", config.hue_red_h)
        # rospy.loginfo("saturation_red_l : %d", config.saturation_red_l)
        # rospy.loginfo("saturation_red_h : %d", config.saturation_red_h)
        # rospy.loginfo("lightness_red_l : %d", config.lightness_red_l)
        # rospy.loginfo("lightness_red_h : %d", config.lightness_red_h)

        # rospy.loginfo("hue_yellow_l : %d", config.hue_yellow_l)
        # rospy.loginfo("hue_yellow_h : %d", config.hue_yellow_h)
        # rospy.loginfo("saturation_yellow_l : %d", config.saturation_yellow_l)
        # rospy.loginfo("saturation_yellow_h : %d", config.saturation_yellow_h)
        # rospy.loginfo("lightness_yellow_l : %d", config.lightness_yellow_l)
        # rospy.loginfo("lightness_yellow_h : %d", config.lightness_yellow_h)

        # rospy.loginfo("hue_green_l : %d", config.hue_green_l)
        # rospy.loginfo("hue_green_h : %d", config.hue_green_h)
        # rospy.loginfo("saturation_green_l : %d", config.saturation_green_l)
        # rospy.loginfo("saturation_green_h : %d", config.saturation_green_h)
        # rospy.loginfo("lightness_green_l : %d", config.lightness_green_l)
        # rospy.loginfo("lightness_green_h : %d", config.lightness_green_h)

        self.get_logger().info('[Detect Traffic Light] Detect Traffic Light Calibration Parameter reconfigured to')
        self.get_logger().info(f'hue_red_l : {config.hue_red_l}')
        self.get_logger().info(f'hue_red_h : {config.hue_red_h}')
        self.get_logger().info(f'saturation_red_l : {config.saturation_red_l}')
        self.get_logger().info(f'saturation_red_h : {config.saturation_red_h}')
        self.get_logger().info(f'lightness_red_l : {config.lightness_red_l}')
        self.get_logger().info(f'lightness_red_h : {config.lightness_red_h}')

        self.get_logger().info(f'hue_yellow_l : {config.hue_yellow_l}')
        self.get_logger().info(f'hue_yellow_h : {config.hue_yellow_h}')
        self.get_logger().info(f'saturation_yellow_l : {config.saturation_yellow_l}')
        self.get_logger().info(f'saturation_yellow_h : {config.saturation_yellow_h}')
        self.get_logger().info(f'lightness_yellow_l : {config.lightness_yellow_l}')
        self.get_logger().info(f'lightness_yellow_h : {config.lightness_yellow_h}')

        self.get_logger().info(f'hue_green_l : {config.hue_green_l}')
        self.get_logger().info(f'hue_green_h : {config.hue_green_h}')
        self.get_logger().info(f'saturation_green_l : {config.saturation_green_l}')
        self.get_logger().info(f'saturation_green_h : {config.saturation_green_h}')
        self.get_logger().info(f'lightness_green_l : {config.lightness_green_l}')
        self.get_logger().info(f'lightness_green_h : {config.lightness_green_h}')

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
        #self.get_logger().info('[Detect Traffic Light] Image received')
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
        #self.get_logger().info('[Detect Traffic Light] Find Traffic Light')
        cv_image_mask = self.fnMaskGreenTrafficLight()
        cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)
        # 定义膨胀核
        kernel = np.ones((3,3),np.uint8)
        # 执行膨胀操作
        cv_image_mask = cv2.dilate(cv_image_mask, kernel, iterations=2)
        status1 = self.fnFindCircleOfTrafficLight(cv_image_mask, 'green')
        #self.get_logger().info(str(status1))
        if status1 == 1 or status1 == 5:
            self.stop_count = 0
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
                #self.get_logger().info(str(status3))
                if status3 == 3:
                    self.red_count += 1
                elif status3 == 4:
                    self.red_count = 0
                    self.stop_count += 1
                else:
                    self.red_count = 0
                    self.stop_count = 0

        if self.green_count > 20:
            #self.get_logger().info("11111111")
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.12
            self.pub_max_vel.publish(msg_pub_max_vel)
            self.get_logger().info("GREEN")
            cv2.putText(self.cv_image,"GREEN", (self.point_col, self.point_low), cv2.FONT_HERSHEY_DUPLEX, 0.5, (80, 255, 0))

        if self.yellow_count > 12:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.06 if not self.off_traffic else 0.12
            self.pub_max_vel.publish(msg_pub_max_vel)
            self.get_logger().info("YELLOW")
            cv2.putText(self.cv_image,"YELLOW", (self.point_col, self.point_low), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 255))

        if self.red_count > 8:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.03
            self.pub_max_vel.publish(msg_pub_max_vel)
            self.get_logger().info("RED")
            cv2.putText(self.cv_image,"RED", (self.point_col, self.point_low), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255))

        if self.stop_count > 8:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.0
            self.pub_max_vel.publish(msg_pub_max_vel)  
            self.get_logger().info("STOP")
            self.off_traffic = True
            cv2.putText(self.cv_image,"STOP", (self.point_col, self.point_low), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255))
        # cv2.namedWindow("Parameters")
        # #cv2.resizeWindow("Parameters", 640, 320);
        # cv2.moveWindow("Parameters",20,20)
        #cv2.imshow("traffic", self.cv_image)
        #cv2.waitkey(3)
        if self.pub_image_type == "compressed":
            # publishes traffic light image in compressed type
            self.pub_image_traffic_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(self.cv_image, "jpg"))

        elif self.pub_image_type == "raw":
            # publishes traffic light image in raw type
            self.pub_image_traffic_light.publish(self.cvBridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        self.display_image()

    def fnMaskRedTrafficLight(self):
        #self.get_logger().info('[Detect Traffic Light] Mask Red Traffic Light')
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
        #self.get_logger().info('[Detect Traffic Light] Mask Yellow Traffic Light')
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
        #self.get_logger().info('[Detect Traffic Light] Mask Green Traffic Light')
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
        #self.get_logger().info('[Detect Traffic Light] Find Circle of Traffic Light')
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
                if find_color == 'green':
                    status = 1
                elif find_color == 'yellow':
                    status = 2
                elif find_color == 'red':
                    status = 3
            elif self.point_col > col2 and self.point_col < col3 and self.point_low > low1 and self.point_low < low3:
                if find_color == 'red':
                    status = 4
                elif find_color == 'green':
                    status = 5
            else:
                status = 6

        return status

    def cbTrafficLightFinished(self, traffic_light_finished_msg):
        #self.get_logger().info('[Detect Traffic Light] Callback Traffic Light Finished')
        self.is_traffic_light_finished = True

    def display_image(self):
    # 确保cv2窗口不会阻止其他操作
        cv2.namedWindow('Traffic Light Detection', cv2.WINDOW_NORMAL)
        cv2.imshow('Traffic Light Detection', self.cv_image)
        cv2.waitKey(3)  # 等待1毫秒，这允许刷新图像但不阻塞执行
    # 如果你想在循环中显示图像并且希望用户能够关闭窗口，
    # 你可能想要检查cv2.getWindowProperty()的返回值以确定窗口是否仍然打开。
    # 这样做可以避免在用户关闭窗口后继续尝试更新窗口，从而引发错误。
        # if cv2.getWindowProperty('Traffic Light Detection', cv2.WND_PROP_VISIBLE) < 1:
        #     cv2.destroyWindow('Traffic Light Detection')

def main(args=None):
    rclpy.init(args=args)
    traffic_light_detector = DetectTrafficLight()
    rclpy.spin(traffic_light_detector)
    #traffic_light_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()