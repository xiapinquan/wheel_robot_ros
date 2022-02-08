#!/usr/bin/env python
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy

temp=1

file_path_1 = "/media/wheeltec/U_PAN1/9.8/"
file_path_2 = ".jpg"
i = 3500


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        # 订阅usb摄像头
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("cv_bridge_image", Image, self.image_callback)

    

    def image_callback(self, msg):
        global temp
        global file_path_1
        global file_path_2
        global i

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        temp = temp +1
        if temp > 50000:
            temp  = 1
        a = temp % 10
        if a ==0:
            file_path = file_path_1 + str(i) + file_path_2
            retval = cv2.imwrite(file_path, image)
            if retval == True:
                i = i + 1
                print(i)
            
        #cv2.imshow("pic", mask)
        #cv2.waitKey(1)
       


rospy.init_node("picture")
follower = Follower()
rospy.spin()

