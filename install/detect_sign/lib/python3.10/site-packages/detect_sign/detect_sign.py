#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Leon Jung, Gilbert

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

class DetectSign(Node):
    def __init__(self):
        super().__init__('detect_sign')

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        self.fnPreproc()

        self.sub_image_type = "compressed" # you can choose image type "compressed", "raw"
        self.pub_image_type = "compressed" # you can choose image type "compressed", "raw"

        if self.sub_image_type == "compressed":
            print("3333")
            # subscribes compressed image
            self.sub_image_original = self.create_subscription(CompressedImage, '/oakd/rgb/preview/image_raw/compressed', self.cbFindTrafficSign, QoSProfile(depth=1))
        elif self.sub_image_type == "raw":
            # subscribes raw image
            self.sub_image_original = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.cbFindTrafficSign, QoSProfile(depth=1))

        self.pub_traffic_sign = self.create_publisher(UInt8, '/detect/traffic_sign', QoSProfile(depth=1))

        if self.pub_image_type == "compressed":
            # publishes traffic sign image in compressed type 
            self.pub_image_traffic_sign = self.create_publisher(CompressedImage, '/detect/image_output/compressed', QoSProfile(depth=1))
        elif self.pub_image_type == "raw":
            # publishes traffic sign image in raw type
            self.pub_image_traffic_sign = self.create_publisher(Image, '/detect/image_output', QoSProfile(depth=1))

        self.cvBridge = CvBridge()

        self.TrafficSign = Enum('TrafficSign', 'divide stop parking tunnel')

        self.counter = 1
        print("1111")

    def fnPreproc(self):
        # Initiate SIFT detector
        #self.sift = cv2.xfeatures2d.SIFT_create()
        # self.sift = cv2.SIFT_create()

        # dir_path = os.path.dirname(os.path.realpath(__file__))
        # dir_path = dir_path.replace('turtlebot3_autorace_detect/nodes', 'turtlebot3_autorace_detect/')
        # dir_path += 'file/detect_sign/'
        
        self.sift = cv2.SIFT_create()

        #dir_path储存了图片文件的目录
        dir_path = "/home/liu/ros/rosTest2/parking_picture"

        # print(dir_path)
        # print(dir_path+ '/parking_not_allowed.png')

        self.img2 = cv2.imread(dir_path+ '/parking_not_allowed.png',0)         # trainImage1
        self.img3 = cv2.imread(dir_path+ '/parking_allowed.png',0)      # trainImage2
        #self.img4 = cv2.imread(dir_path+ '/tunnel.png',0)       # trainImage3

        if self.img2 is None or self.img3 is None :
            print("One or more images are empty. Please check the image paths.")
            return
        # print(self.img3)

        self.kp2, self.des2 = self.sift.detectAndCompute(self.img2,None)
        self.kp3, self.des3 = self.sift.detectAndCompute(self.img3,None)  # 允许停车
        #self.kp4, self.des4 = self.sift.detectAndCompute(self.img4,None)

        print("44444444")

    def fnCalcMSE(self, arr1, arr2):
            squared_diff = (arr1 - arr2) ** 2
            sum = np.sum(squared_diff)
            num_all = arr1.shape[0] * arr1.shape[1] #cv_image_input and 2 should have same shape
            err = sum / num_all
            return err

    def cbFindTrafficSign(self, image_msg):
        
        self.get_logger().info("hello!!")

        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
            # np_arr = np.fromstring(image_msg.data, np.uint8)
            np_arr = np.frombuffer(image_msg.data,np.uint8)
            cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        MIN_MATCH_COUNT = 6      # 更改参数调节
        MIN_MSE_DECISION = 50000

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input,None)

        matches2 = self.flann.knnMatch(des1,self.des2,k=2)
        matches3 = self.flann.knnMatch(des1,self.des3,k=2)
        # matches4 = self.flann.knnMatch(des1,self.des4,k=2)

        image_out_num = 1

        good2 = []
        for m,n in matches2:
            if m.distance < 0.7*n.distance:
                good2.append(m)

        if len(good2)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good2 ]).reshape(-1,1,2)
            dst_pts = np.float32([ self.kp2[m.trainIdx].pt for m in good2 ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask2 = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.stop.value

                self.pub_traffic_sign.publish(msg_sign)

                self.get_logger().info("TrafficSign 2")

                image_out_num = 2

        else:
            matchesMask2 = None

        good3 = []
        for m,n in matches3:
            if m.distance < 0.7*n.distance:
                good3.append(m)
        image_match = cv2.drawMatches(cv_image_input,kp1,self.img3,self.kp3,good3,None)  ## 使用 drawMatches 函数绘制匹配的关键点
        print(len(good3))
        self.get_logger().info("len(good3):%d" %len(good3))
        if len(good3)> MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good3 ]).reshape(-1,1,2)
            dst_pts = np.float32([ self.kp3[m.trainIdx].pt for m in good3 ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask3 = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            mse_text = "MSE: {:.2f}".format(mse) #将 MSE 转换为字符串
            # 使用 putText 函数将 MSE 写在图像上
            cv2.putText(image_match, mse_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            # 显示图像
            cv2.imshow('Matches', image_match)
            cv2.waitKey(3)
            # cv2.destroyAllWindows()
            self.get_logger().info("mse:%d" %mse) # //*print log
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.parking.value

                self.pub_traffic_sign.publish(msg_sign)

                print("TrafficSign 3")

                image_out_num = 3

        else:
            matchesMask3 = None

        # good4 = []
        # for m,n in matches4:
        #     if m.distance < 0.7*n.distance:
        #         good4.append(m)
        # if len(good4)>MIN_MATCH_COUNT:
        #     src_pts = np.float32([ kp1[m.queryIdx].pt for m in good4 ]).reshape(-1,1,2)
        #     dst_pts = np.float32([ self.kp4[m.trainIdx].pt for m in good4 ]).reshape(-1,1,2)

        #     M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        #     matchesMask4 = mask.ravel().tolist()

        #     mse = self.fnCalcMSE(src_pts, dst_pts)
        #     if mse < MIN_MSE_DECISION:
        #         msg_sign = UInt8()
        #         msg_sign.data = self.TrafficSign.tunnel.value

        #         self.pub_traffic_sign.publish(msg_sign)

        #         self.get_logger().info("TrafficSign 4")

        #         image_out_num = 4

        # else:
        #    matchesMask4 = None

        if image_out_num == 1:
            if self.pub_image_type == "compressed":
                # publishes traffic sign image in compressed type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_input, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes traffic sign image in raw type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(cv_image_input, "bgr8"))

        elif image_out_num == 2:
            draw_params2 = dict(matchColor = (0,0,255), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matchesMask2, # draw only inliers
                            flags = 2)

            final2 = cv2.drawMatches(cv_image_input,kp1,self.img2,self.kp2,good2,None,**draw_params2)

            if self.pub_image_type == "compressed":
                # publishes traffic sign image in compressed type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(final2, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes traffic sign image in raw type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(final2, "bgr8"))

        elif image_out_num == 3:
            draw_params3 = dict(matchColor = (255,0,0), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matchesMask3, # draw only inliers
                            flags = 2)

            final3 = cv2.drawMatches(cv_image_input,kp1,self.img3,self.kp3,good3,None,**draw_params3)

            if self.pub_image_type == "compressed":
                # publishes traffic sign image in compressed type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(final3, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes traffic sign image in raw type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(final3, "bgr8"))

        # elif image_out_num == 4:
        #     draw_params4 = dict(matchColor = (255,0,0), # draw matches in green color
        #                     singlePointColor = None,
        #                     matchesMask = matchesMask4, # draw only inliers
        #                     flags = 2)

        # final4 = cv2.drawMatches(cv_image_input,kp1,self.img4,self.kp4,good4,None,**draw_params4)

            # if self.pub_image_type == "compressed":
            #     # publishes traffic sign image in compressed type
            #     self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(final4, "jpg"))

            # elif self.pub_image_type == "raw":
            #     # publishes traffic sign image in raw type
            #     self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(final4, "bgr8"))
        # print("2222222")
    # def main(self):
    #     rospy.spin()

# if __name__ == '__main__':
#     rospy.init_node('detect_sign')
#     node = DetectSign()
#     node.main()

def main(args=None):
    rclpy.init(args=args)
    sign_detector = DetectSign()
    rclpy.spin(sign_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



















