#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import rospy
import message_filters
import numpy as np
import cv2
from matplotlib import pyplot as plt
from cv_bridge import CvBridge 

from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import ObjectCount

from sensor_msgs.msg import Image
from put_out_fire.msg import position as PositionMsg
from std_msgs.msg import String as StringMsg
from std_msgs.msg import Int8
from dynamic_reconfigure.server import Server
from simple_follower.cfg import Params_colorConfig

np.seterr(all='raise')  

plt.close('all')
class visualTracker:
	def __init__(self):
		self.bridge = CvBridge()
		self.i=0		#语音识别标志

		self.targetUpper = np.array(rospy.get_param('~targetred/upper'))
		self.targetLower = np.array(rospy.get_param('~targetred/lower'))

		self.col_red_U =np.array(rospy.get_param('~targetred/upper'))# red
		self.col_red_L =np.array(rospy.get_param('~targetred/lower'))

		self.col_blue_U =np.array(rospy.get_param('~targetblue/upper'))# blue
		self.col_blue_L =np.array(rospy.get_param('~targetblue/lower'))

		self.col_green_U =np.array(rospy.get_param('~targetgreen/upper'))# green
		self.col_green_L = np.array(rospy.get_param('~targetgreen/lower'))
		 
		self.col_yellow_U =np.array(rospy.get_param('~targetyellow/upper')) #yellow
		self.col_yellow_L =np.array(rospy.get_param('~targetyellow/lower'))

		self.pictureHeight= rospy.get_param('~pictureDimensions/pictureHeight')
		self.pictureWidth = rospy.get_param('~pictureDimensions/pictureWidth')
		vertAngle =rospy.get_param('~pictureDimensions/verticalAngle')
		horizontalAngle =  rospy.get_param('~pictureDimensions/horizontalAngle')
		# precompute tangens since thats all we need anyways: 预计算切线
		self.tanVertical = np.tan(vertAngle)
		self.tanHorizontal = np.tan(horizontalAngle)	
		self.lastPoCsition =None

		self.targetDist = rospy.get_param('~targetDist')

		# one callback that deals with depth and rgb at the same time 一个同时处理深度和RGB的回调
		im_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
		dep_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
		fire_sub = message_filters.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes)

		fire_status_sub = rospy.Subscriber('/darknet_ros/found_object', ObjectCount, self.checkFireStatus)

		self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub,dep_sub,fire_sub], 10, 0.5)
		
		self.timeSynchronizer.registerCallback(self.trackObject)

		self.positionPublisher = rospy.Publisher('/object_tracker/current_position', PositionMsg, queue_size=3)
		#self.infoPublisher = rospy.Publisher('/object_size=3)

		self.color_obj = Server(Params_colorConfig,self.colorreconfigure)

		rospy.logwarn(self.targetUpper)

	def trackObject(self, image_data, depth_data,fire_data):
		if(image_data.encoding != 'rgb8'):
			raise ValueError('image is not rgb8 as expected')
		#convert both images to numpy arrays # 将两张图片都转换为Numpy数组
		frame = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='rgb8')
		depthFrame = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')#"32FC1")	
		if(np.shape(frame)[0:2] != (self.pictureHeight, self.pictureWidth)):
			raise ValueError('image does not have the right shape. shape(frame): {}, shape parameters:{}'.format(np.shape(frame)[0:2], (self.pictureHeight, self.pictureWidth)))

		newPos = None
		contours = []
		for box in fire_data.bounding_boxes:
			print("xpq ,boxes=",box.Class,",xmin=",box.xmin,",ymin=",box.ymin,",xmax=",box.xmax,",ymax=",box.ymax)
			contours.append(self.createContour(box))
		print("xpq  type(contours)",np.shape(contours))

		try:
			# go threw all the contours. starting with the bigest one  抛出了所有的轮廓线。从最大的那个开始
			contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]
			# get position of object for this contour 获取此轮廓的对象位置
		 	pos = self.analyseContour(contour, depthFrame)
			# if it's the first one we found it will be the fall back for the next scan if we don't find a plausible one
			#如果这是我们发现的第一个，如果我们找不到可信的，它将是下一次扫描的后备
			if newPos is None:
				newPos = pos
			# check if the position is plausible
			#if self.checkPosPlausible(pos):
			#检查位置是否合理。

			#if self.checkPosPlausible(位置)：
			self.lastPosition = pos
			self.publishPosition(pos)
			return
			#我们没有找到可信的最后一个位置，所以我们只保存了最大的等高线
			self.lastPosition = newPos #we didn't find a plossible last position, so we just save the biggest contour 
		except IndexError:
			# and publish warnings
			# 打印警告信息
			rospy.logwarn('no position found')
			posMsg = PositionMsg(0, 0,self.targetDist)
			self.positionPublisher.publish(posMsg)

	fireStatusCount = 0
	carStopFlag = 0
	def checkFireStatus(self,statusMsg):
		print("xpq , statusMsg = ",statusMsg.count)
		if(statusMsg.count <= 0):
			self.fireStatusCount +=1
		else:
			self.fireStatusCount = 0

		if(self.fireStatusCount > 20):
			rospy.logwarn('current is no fire object , car stoping!')
			posMsg = PositionMsg(0, 0,self.targetDist)
			self.positionPublisher.publish(posMsg)
			self.fireStatusCount = 0


	def createContour(self, box):
		pixels = 2*(box.xmax + box.ymax - box.xmin - box.ymin + 1)
		xmin = box.xmin
		ymin = box.ymin
		xmax = box.xmax
		ymax = box.ymax
		contour = np.ones([pixels,1,2],np.int32)
		#print("createContour",np.shape(contour))
		for index in range(contour.size):
			if(index < pixels/2):
				if(ymin < ymax):
					contour[index][0] = [xmin,ymin]
					ymin += 1
					continue
				if(xmin < xmax):
					contour[index][0] = [xmin,ymin]
					xmin += 1
			elif(index > pixels/2):
				if(ymin < ymax):			
					contour[index][0] = [xmax,ymax]
					ymax -= 1
					continue
				if(xmin < xmax):
					contour[index][0] = [xmax,ymax]
					xmax -= 1
			else:
				xmin = box.xmin
				ymin = box.ymin
				contour[index][0] = [xmax,ymax]
				contour[index-1][0] = [xmax,ymax-1]
		return contour


	def publishPosition(self, pos):
		# calculate the angles from the raw position
		#从原始位置计算角度
		angleX = self.calculateAngleX(pos)
		angleY = self.calculateAngleY(pos)
		# publish the position (angleX, angleY, distance)
		posMsg = PositionMsg(angleX, angleY, pos[1])
		self.positionPublisher.publish(posMsg)
		
		
	def calculateAngleX(self, pos):
		'''calculates the X angle of displacement from straight ahead'''
		'''“检查某个位置是否合理，即是否足够接近上一个位置。”'''
		centerX = pos[0][0]
		displacement = 2*centerX/self.pictureWidth-1
		angle = -1*np.arctan(displacement*self.tanHorizontal)
		return angle

	def calculateAngleY(self, pos):
		'''calculates the X angle of displacement from straight ahead从正前方计算位移的X角度'''
		centerY = pos[0][1]
		displacement = 2*centerY/self.pictureHeight-1
		angle = -1*np.arctan(displacement*self.tanVertical)
		return angle
	
	def analyseContour(self, contour, depthFrame):
		'''Calculates the centers coordinates and distance for a given contour
		计算给定等高线的中心坐标和距离
		Args:
			contour (opencv contour): contour of the object
			depthFrame (numpy array): the depth image
		
		Returns:
			centerX, centerY (doubles): center coordinates
			averageDistance : distance of the object

		参数：
        	轮廓(OpenCV轮廓)：对象的轮廓。
        	Deep thFrame(Numpy Array)：深度图像。
        返回：
        	中心X、中心Y(双精度)：中心坐标。
        	AverageDistance：对象的距离
		'''
		# get a rectangle that completely contains the object
		# 获取完全包含该对象的矩形
		centerRaw, size, rotation = cv2.minAreaRect(contour)

		# get the center of that rounded to ints (so we can index the image)
		center = np.round(centerRaw).astype(int)

		# find out how far we can go in x/y direction without leaving the object (min of the extension of the bounding rectangle/2 (here 3 for safety)) 
		# 找出在不离开对象的情况下，我们可以在x/y方向上走多远(最小边界矩形的延伸长度/2(为安全起见，此处为3))
		minSize = int(min(size)/3)

		# get all the depth points within this area (that is within the object)
		# 获取该区域内（即对象内）的所有深度点
		depthObject = depthFrame[(center[1]-minSize):(center[1]+minSize), (center[0]-minSize):(center[0]+minSize)]

		# get the average of all valid points (average to have a more reliable distance measure)
		# 获得所有有效点的平均值（平均值以获得更可靠的距离度量）
		depthArray = depthObject[~np.isnan(depthObject)]
		averageDistance = np.mean(depthArray)
		if(averageDistance>400 or averageDistance<3000):
			pass
		else:
			averageDistance=400

		if len(depthArray) == 0:
			rospy.logwarn('empty depth array. all depth values are nan')


		return (centerRaw, averageDistance)
	# Dynamic parameter configuration
	# 动态参数配置
	def colorreconfigure(self, config, level):

		self.color = config.color

		if self.color== 0:  #

			HSV_H_MIN =config.HSV_H_MIN
			HSV_S_MIN =config.HSV_S_MIN
			HSV_V_MIN =config.HSV_V_MIN
			HSV_H_MAX =config.HSV_H_MAX
			HSV_S_MAX =config.HSV_S_MAX
			HSV_V_MAX =config.HSV_V_MAX
			self.targetUpper=np.array([HSV_H_MIN,HSV_S_MIN,HSV_V_MIN])
			self.targetLower=np.array([HSV_H_MAX,HSV_S_MAX,HSV_V_MAX])

		elif self.color == 1:  #

			self.targetUpper=self.col_red_U
			self.targetLower=self.col_red_L

		elif self.color== 2:  #

			self.targetUpper=self.col_blue_U
			self.targetLower=self.col_blue_L
		elif self.color== 3:  #

			self.targetUpper=self.col_green_U
			self.targetLower=self.col_green_L

		elif self.color== 4:  #

			self.targetUpper=self.col_yellow_U
			self.targetLower=self.col_yellow_L

		return  config

if __name__ == '__main__':
	visualfwflagPublisher = rospy.Publisher('/visual_follow_flag', Int8, queue_size =1)
	rospy.init_node('visual_tracker',anonymous = False)
	tracker=visualTracker()
	rospy.logwarn('visualTracker init done')

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn('failed')
