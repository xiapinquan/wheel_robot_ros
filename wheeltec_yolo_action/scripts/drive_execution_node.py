#!/usr/bin/env python
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from simple_follower.msg import position
from wheeltec_yolo_action.msg import CtrlData

last_erro=0

mask_x_left = 1.0000
mask_x_right = 1.0000
mask_y_top = 1.0000
mask_y_bot = 1.0000
center_target = 0.5
vel_x = 0.0000
vel_y_P = 0.0000
vel_y_D = 0.0000
vel_z_P = 0.0000
vel_z_D = 0.0000
en = 0

def nothing(s):
    pass
col_black = (0,0,0,180,255,46)# black
col_red = (0,50,60,14,255,255)# red
col_blue = (100,43,46,124,255,255)# blue
col_green= (35,43,46,77,255,255)# green
col_yellow = (26,43,46,34,255,255)# yellow

# cv2.namedWindow('Adjust_hsv',cv2.WINDOW_NORMAL)
# Switch = '0:Red\n1:Green\n2:Blue\n3:Yellow\n4:Black'
# cv2.createTrackbar(Switch,'Adjust_hsv',0,4,nothing)


# cv2.createTrackbar('LH','Adjust_hsv',0,180,nothing)
# cv2.createTrackbar('LS','Adjust_hsv',0,255,nothing)
# cv2.createTrackbar('LV','Adjust_hsv',0,255,nothing)
# cv2.createTrackbar('HH','Adjust_hsv',0,180,nothing)
# cv2.createTrackbar('HS','Adjust_hsv',0,255,nothing)
# cv2.createTrackbar('HV','Adjust_hsv',0,255,nothing)


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        # 订阅usb摄像头
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("cv_bridge_image", Image, self.image_callback)

        self.ctrl_sub = rospy.Subscriber("/ctrl_data", CtrlData, self.ctrl_callback)
        self.decelerate_sub = rospy.Subscriber("/decelerate_data", Int8, self.decelerate_callback)
        #self.position_sub = rospy.Subscriber("/object_tracker/current_position", position, self.position_callback)

        # 订阅深度相机
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image,self.image_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.drive_line_pub = rospy.Publisher("/drive_line", Image, queue_size=1)
        self.control_flag_pub = rospy.Publisher("/control_flag", Int8, queue_size=1)
        self.twist = Twist()
        self.flag = Int8()
        self.flag.data = 1;

    def image_callback(self, msg):
        global last_erro
        global mask_x_left
        global mask_x_right
        global mask_y_top
        global mask_y_bot
        global center_target
        global vel_x
        global vel_y_P
        global vel_y_D
        global vel_z_P
        global vel_z_D
        global en
		
        image1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #h, w, d = image.shape
        image = cv2.resize(image1, (320,240), interpolation=cv2.INTER_AREA)#提高帧率
        #h, w, d = image.shape
        # hsv将RGB图像分解成色调H，饱和度S，明度V
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #hsv1 = cv2.cvtColor(image1, cv2.COLOR_BGR2HSV)
        # for i in range(0,h):
        #     for j in range(0,w):
        #         if 160 < hsv.item(i,j,0) < 180:
        #             hsv.itemset((i,j,0),5)
        #         j = j + 1
        #     i = i + 1 

        # 颜色的范围        # 第二个参数：lower指的是图像中低于这个lower的值，图像值变为0
        # 第三个参数：upper指的是图像中高于这个upper的值，图像值变为0
        # 而在lower～upper之间的值变成255
        # kernel = numpy.ones((5,5),numpy.uint8)
        # hsv_erode = cv2.erode(hsv,kernel,iterations=1)
        # hsv_dilate = cv2.dilate(hsv_erode,kernel,iterations=1)

        
        # lowerbH=cv2.getTrackbarPos('LH','Adjust_hsv')
        # lowerbS=cv2.getTrackbarPos('LS','Adjust_hsv')
        # lowerbV=cv2.getTrackbarPos('LV','Adjust_hsv')
        # upperbH=cv2.getTrackbarPos('HH','Adjust_hsv')
        # upperbS=cv2.getTrackbarPos('HS','Adjust_hsv')
        # upperbV=cv2.getTrackbarPos('HV','Adjust_hsv')

        
        lowerbH=col_red[0]
        lowerbS=col_red[1]
        lowerbV=col_red[2]
        upperbH=col_red[3]
        upperbS=col_red[4]
        upperbV=col_red[5]


        # mask=cv2.inRange(hsv,(lowerbH,lowerbS,lowerbV),(upperbH,upperbS,upperbV))
        mask1=cv2.inRange(hsv,(lowerbH,lowerbS,lowerbV),(upperbH,upperbS,upperbV))
        mask2=cv2.inRange(hsv,(lowerbH+165,lowerbS,lowerbV),(upperbH+170,upperbS,upperbV))
        mask = cv2.bitwise_or(mask1, mask2)

        # 在图像某处绘制一个指示，因为只考虑20行宽的图像，所以使用numpy切片将以外的空间区域清空
        h, w, d = image.shape       
        search_top = h*mask_y_top
        search_bot = h*mask_y_bot
        mask[0:int(search_top), 0:w] = 0
        mask[int(search_bot):h, 0:w] = 0
        mask[0:h, 0:int(mask_x_left*w)] = 0
        mask[0:h, int(mask_x_right*w):w] = 0
        
        # 计算mask图像的重心，即几何中心
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 10, (255, 0, 255), -1)
            #cv2.circle(image, (cx-75, cy), 10, (0, 0, 255), -1)
            #cv2.circle(image, (w/2, h), 10, (0, 255, 255), -1)
            if cv2.circle:
            # 计算图像中心线和目标指示线中心的距离
                erro = cx - w*center_target
                d_erro=erro-last_erro
                
                self.twist.linear.x = vel_x
                

                if erro!=0:
                    self.twist.angular.z = -float(erro)*vel_z_P-float(d_erro)*vel_z_D
                    self.twist.linear.y = float(erro)*vel_y_P-float(d_erro)*vel_y_D
                    if self.twist.linear.y > 0.2:
						self.twist.linear.y = 0.2
                else :
                    self.twist.angular.z = 0
                    self.twist.linear.y = 0 
                last_erro=erro
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
        if en > 0:
            self.cmd_vel_pub.publish(self.twist)
            if en == 1:
                cv2.putText(mask, "outside_left", (0,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255),1)
            elif en == 2:
                cv2.putText(mask, "outside_right", (0,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255),1)
            elif en == 3:
                cv2.putText(mask, "inside_left", (0,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255),1)
            elif en == 4:
                cv2.putText(mask, "inside_right", (0,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255),1)
        else :
            cv2.putText(mask, "no_line_init", (0,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255),1)

        # cv2.imshow("window", image)
        # cv2.imshow("window1", image1)
        #cv2.imshow("window2", hsv_erode)
        #cv2.imshow("window3", hsv_dilate)
        #cv2.imshow("window4", mask)
        cv2.imshow("Adjust_hsv", mask)
        # cv2.imshow("Adjust_hsv1", hsv)
        # cv2.imshow("Adjust_hsv2", mask1)
        # cv2.imshow("Adjust_hsv3", mask2)
        cv2.waitKey(1)
        #self.drive_line_pub.publish(self.bridge.cv2_to_imgmsg(mask,'mono8'))
        self.control_flag_pub.publish(self.flag)


    def ctrl_callback(self, msg): 
        global mask_x_left
        global mask_x_right
        global mask_y_top
        global mask_y_bot
        global center_target
        global vel_x
        global vel_y_P
        global vel_y_D
        global vel_z_P
        global vel_z_D
        global en
        mask_x_left = msg.mask_x_left
        mask_x_right = msg.mask_x_right
        mask_y_top = msg.mask_y_top
        mask_y_bot = msg.mask_y_bot
        center_target = msg.center_target
        vel_x = msg.vel_x
        vel_y_P = msg.vel_y_P
        vel_y_D = msg.vel_y_D
        vel_z_P = msg.vel_z_P
        vel_z_D = msg.vel_z_D
        en  = msg.en

    def  decelerate_callback(self, msg): 
        global vel_x
        vel_x = float(msg.data)/100
        # print("decelerate")
        # print(vel_x)


rospy.init_node("opencv")
follower = Follower()
rospy.spin()



