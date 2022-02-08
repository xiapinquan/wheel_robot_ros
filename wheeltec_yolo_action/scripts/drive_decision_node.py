#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy
from simple_follower.msg import position
from wheeltec_yolo_action.msg import CtrlData
import threading
import time
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist

from dynamic_reconfigure.server import Server
from wheeltec_yolo_action.cfg import paramsConfig


out_l_mask_x_left = 0
out_l_mask_x_right = 0.5
out_l_mask_y_top = 0.5
out_l_mask_y_bot = 1
out_l_center_target = 0.20
out_l_vel_x = 0.15
out_l_vel_y_P = 0
out_l_vel_y_D = 0
out_l_vel_z_P = 0.01
out_l_vel_z_D = 0.001

out_r_mask_x_left = 0.5
out_r_mask_x_right = 1
out_r_mask_y_top = 0.5
out_r_mask_y_bot = 1
out_r_center_target = 0.75
out_r_vel_x = 0.15
out_r_vel_y_P = 0
out_r_vel_y_D = 0
out_r_vel_z_P = 0.01
out_r_vel_z_D = 0.001

in_l_mask_x_left = 0
in_l_mask_x_right = 0.5
in_l_mask_y_top = 0.66667
in_l_mask_y_bot = 1
in_l_center_target = 0.28
in_l_vel_x = 0.1
in_l_vel_y_P = 0.001
in_l_vel_y_D = 0.003
in_l_vel_z_P = 0.006
in_l_vel_z_D = 0.001

in_r_mask_x_left = 0.5
in_r_mask_x_right = 1
in_r_mask_y_top = 0.66667
in_r_mask_y_bot = 1
in_r_center_target = 0.75
in_r_vel_x = 0.1
in_r_vel_y_P = 0.001
in_r_vel_y_D = 0.003
in_r_vel_z_P = 0.006
in_r_vel_z_D = 0.001



col_red = (0,70,60,10,255,255)# red
col_blue = (100,43,46,124,255,255)# blue
col_green= (50,55,65,85,255,255)# green

side_flag = 0
pos_angleX = 0.0000
road_construction_flag = 0
crossing_flag = 0
bus_flag = 0
stop_flag = 0
stop_x = 0
stop_y = 0
last_erro_x = 0
last_erro_y = 0
control_flag = 0
last_currentState = 0
last_side = 0

ctrl_pub = rospy.Publisher("/ctrl_data", CtrlData, queue_size=1)

def thread_job():
	rospy.spin()
	
def position_callback(msg):
	global pos_angleX
	if msg.distance < 0.4:
		pos_angleX = msg.angleX
		

def control_flag_callback(msg):
	global control_flag
	control_flag = msg.data



def side_flag_callback(msg):
	global side_flag
	global road_construction_flag
	global crossing_flag
	global bus_flag
	global stop_flag
	global stop_x
	global stop_y
	
	for boxes in msg.bounding_boxes:
		if boxes.id == 0 and boxes.probability > 0.6:
			if boxes.xmax < 320 and boxes.ymax > 240 and boxes.xmax > 30:
				side_flag = 1
			elif boxes.xmin > 320 and boxes.ymax > 240 and boxes.xmin < 610:
				side_flag = 2
		elif boxes.id == 1 and boxes.probability > 0.6:
			if boxes.ymax > 450 and boxes.xmin < 400 and boxes.xmax > 250 :
				road_construction_flag = 1
		elif boxes.id == 2 and boxes.probability > 0.4:
			if 380 < boxes.ymin < 450:
				crossing_flag = 1
		elif boxes.id == 3 and boxes.probability > 0.6:
			if 240 < boxes.ymin < 420:
				bus_flag = 1
		elif boxes.id == 4 and boxes.probability > 0.8:
			if boxes.ymin > 100 and boxes.ymax < 465:
				stop_flag = 1
				stop_x = (boxes.xmin + boxes.xmax)/2
				stop_y = (boxes.ymin + boxes.ymax)/2

	# print(side_flag)
	# print("_____________")

def T_junction():
	cx = 0
	cy = 0
	image = rospy.wait_for_message("/camera/rgb/image_raw", Image, timeout = None)	
	bridge = cv_bridge.CvBridge()
	image = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
	image = cv2.resize(image, (320,240), interpolation=cv2.INTER_AREA)
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	# kernel = numpy.ones((5,5),numpy.uint8)
	# hsv_erode = cv2.erode(hsv,kernel,iterations=1)
	# hsv_dilate = cv2.dilate(hsv_erode,kernel,iterations=1)
	lowerbH=col_green[0]
	lowerbS=col_green[1]
	lowerbV=col_green[2]
	upperbH=col_green[3]
	upperbS=col_green[4]
	upperbV=col_green[5]
	mask=cv2.inRange(hsv,(lowerbH,lowerbS,lowerbV),(upperbH,upperbS,upperbV))

	kernel = numpy.ones((5,5),numpy.uint8)
	hsv_erode = cv2.erode(mask,kernel,iterations=1)
	mask = cv2.dilate(hsv_erode,kernel,iterations=1)

	h, w, d = image.shape
	mask[h*4/5:h, 0:w] = 0
	# mask[0:h/15, 0:w] = 0
	# mask[h/2:h, 0:w] = 0
	# mask[0:h, 0:w/15] = 0
	# mask[0:h, 14/15*w:w] = 0
	# cv2.imshow("test", mask)
	# cv2.waitKey(1)
	M = cv2.moments(mask)
	if M['m00'] > 0:
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
	if cx != 0 and cy != 0 :
		return True
	else:
		return False
	
	
def ctrl_data(currentState, side):
	global out_l_mask_x_left
	global out_l_mask_x_right
	global out_l_mask_y_top
	global out_l_mask_y_bot
	global out_l_center_target
	global out_l_vel_x
	global out_l_vel_y_P
	global out_l_vel_y_D
	global out_l_vel_z_P
	global out_l_vel_z_D

	global out_r_mask_x_left
	global out_r_mask_x_right
	global out_r_mask_y_top
	global out_r_mask_y_bot
	global out_r_center_target
	global out_r_vel_x
	global out_r_vel_y_P
	global out_r_vel_y_D
	global out_r_vel_z_P
	global out_r_vel_z_D

	global in_l_mask_x_left
	global in_l_mask_x_right
	global in_l_mask_y_top
	global in_l_mask_y_bot
	global in_l_center_target
	global in_l_vel_x
	global in_l_vel_y_P
	global in_l_vel_y_D
	global in_l_vel_z_P
	global in_l_vel_z_D

	global in_r_mask_x_left
	global in_r_mask_x_right
	global in_r_mask_y_top
	global in_r_mask_y_bot
	global in_r_center_target
	global in_r_vel_x
	global in_r_vel_y_P
	global in_r_vel_y_D
	global in_r_vel_z_P
	global in_r_vel_z_D
	ctrldata = CtrlData()
	if currentState == 1:
		if side == 1:	#out_le
			ctrldata.mask_x_left = out_l_mask_x_left
			ctrldata.mask_x_right = out_l_mask_x_right
			ctrldata.mask_y_top = out_l_mask_y_top
			ctrldata.mask_y_bot = out_l_mask_y_bot
			ctrldata.center_target = out_l_center_target
			ctrldata.vel_x = out_l_vel_x
			ctrldata.vel_y_P = out_l_vel_y_P
			ctrldata.vel_y_D = out_l_vel_y_D
			ctrldata.vel_z_P = out_l_vel_z_P
			ctrldata.vel_z_D = out_l_vel_z_D
			ctrldata.en = 1
		elif side == 2:	#out_ri
			ctrldata.mask_x_left = out_r_mask_x_left
			ctrldata.mask_x_right = out_r_mask_x_right
			ctrldata.mask_y_top = out_r_mask_y_top
			ctrldata.mask_y_bot = out_r_mask_y_bot
			ctrldata.center_target = out_r_center_target
			ctrldata.vel_x = out_r_vel_x
			ctrldata.vel_y_P = out_r_vel_y_P
			ctrldata.vel_y_D = out_r_vel_y_D
			ctrldata.vel_z_P = out_r_vel_z_P
			ctrldata.vel_z_D = out_r_vel_z_D
			ctrldata.en = 2
	elif currentState == 2:
		if side == 1:	#in_le
			ctrldata.mask_x_left = in_l_mask_x_left
			ctrldata.mask_x_right = in_l_mask_x_right
			ctrldata.mask_y_top = in_l_mask_y_top
			ctrldata.mask_y_bot = in_l_mask_y_bot
			ctrldata.center_target = in_l_center_target
			ctrldata.vel_x = in_l_vel_x
			ctrldata.vel_y_P = in_l_vel_y_P
			ctrldata.vel_y_D = in_l_vel_y_D
			ctrldata.vel_z_P = in_l_vel_z_P
			ctrldata.vel_z_D = in_l_vel_z_D
			ctrldata.en = 3
		elif side == 2:	#in_ri
			ctrldata.mask_x_left = in_r_mask_x_left
			ctrldata.mask_x_right = in_r_mask_x_right
			ctrldata.mask_y_top = in_r_mask_y_top
			ctrldata.mask_y_bot = in_r_mask_y_bot
			ctrldata.center_target = in_r_center_target
			ctrldata.vel_x = in_r_vel_x
			ctrldata.vel_y_P = in_r_vel_y_P
			ctrldata.vel_y_D = in_r_vel_y_D
			ctrldata.vel_z_P = in_r_vel_z_P
			ctrldata.vel_z_D = in_r_vel_z_D
			ctrldata.en = 4
	return ctrldata
	
def line_judgment():
	image = rospy.wait_for_message("/camera/rgb/image_raw", Image, timeout = None)	
	bridge = cv_bridge.CvBridge()
	image = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
	image = cv2.resize(image, (320,240), interpolation=cv2.INTER_AREA)
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	
	lowerbH=col_red[0]
	lowerbS=col_red[1]
	lowerbV=col_red[2]
	upperbH=col_red[3]
	upperbS=col_red[4]
	upperbV=col_red[5]
	mask1=cv2.inRange(hsv,(lowerbH,lowerbS,lowerbV),(upperbH,upperbS,upperbV))
	mask2=cv2.inRange(hsv,(lowerbH+165,lowerbS,lowerbV),(upperbH+170,upperbS,upperbV))
	mask = cv2.bitwise_or(mask1, mask2)
	
	h, w, d = image.shape
	mask[0:h/2, 0:w] = 0
	#cv2.imshow("Adjust_hsv", mask)
	#cv2.waitKey(1)
	#mask[h:h, 0:w] = 0
	M = cv2.moments(mask)
	cx = w/2
	cy = 0
	if M['m00'] > 0:
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
	a = cx - w/2
	
	return a

def reconfigCB(config,level):

	global last_currentState
	global last_side

	global out_l_center_target
	global out_l_vel_z_P
	global out_l_vel_z_D

	global out_r_center_target
	global out_r_vel_z_P
	global out_r_vel_z_D

	global in_l_center_target
	global in_l_vel_y_P
	global in_l_vel_y_D
	global in_l_vel_z_P
	global in_l_vel_z_D

	global in_r_center_target
	global in_r_vel_y_P
	global in_r_vel_y_D
	global in_r_vel_z_P
	global in_r_vel_z_D

	out_l_center_target = config.out_l_center_target
	out_l_vel_z_P = config.out_l_vel_z_P
	out_l_vel_z_D = config.out_l_vel_z_D

	out_r_center_target = config.out_r_center_target
	out_r_vel_z_P = config.out_r_vel_z_P
	out_r_vel_z_D = config.out_r_vel_z_D

	in_l_center_target = config.in_l_center_target
	in_l_vel_y_P = config.in_l_vel_y_P
	in_l_vel_y_D = config.in_l_vel_y_D
	in_l_vel_z_P = config.in_l_vel_z_P
	in_l_vel_z_D = config.in_l_vel_z_D

	in_r_center_target = config.in_r_center_target
	in_r_vel_y_P = config.in_r_vel_y_P
	in_r_vel_y_D = config.in_r_vel_y_D
	in_r_vel_z_P = config.in_r_vel_z_P
	in_r_vel_z_D = config.in_r_vel_z_D 

	if last_currentState != 0:
		ctrldata_pub = ctrl_data(last_currentState, last_side)
		ctrl_pub.publish(ctrldata_pub)

	return config

def control_drive():
	global last_currentState	#1=outside	2=inside
	global last_side			#1=left		2=right
	currentState = 0  #1=outside	2=inside
	side = 0  #1=left		2=right
	i = 0
	temp1 = 0
	temp2 = 0
	temp3 = 0
	count1 = -1
	count2 = -1
	#stop_count = 0
	global side_flag
	global pos_angleX
	global road_construction_flag
	global crossing_flag
	global bus_flag
	global stop_flag
	global stop_x
	global stop_y
	global last_erro_x
	global last_erro_y
	global control_flag

	global out_l_center_target
	global out_l_vel_z_P
	global out_l_vel_z_D
	global out_r_center_target
	global out_r_vel_z_P
	global out_r_vel_z_D
	global in_l_center_target
	global in_l_vel_y_P
	global in_l_vel_y_D
	global in_l_vel_z_P
	global in_l_vel_z_D
	global in_r_center_target
	global in_r_vel_y_P
	global in_r_vel_y_D
	global in_r_vel_z_P
	global in_r_vel_z_D

	ctrldata_pub = CtrlData()
	decelerate = Int8()
	msg = Int8()

	rospy.init_node("control_drive")
	
	add_thread = threading.Thread(target = thread_job)
	add_thread.start
	
	
	decelerate_pub = rospy.Publisher("/decelerate_data", Int8, queue_size=1)
	line_pub = rospy.Publisher("/line_judgment", Int8, queue_size=1)
	cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	#image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
	position_sub = rospy.Subscriber("/object_tracker/current_position", position, position_callback)
	side_flag_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, side_flag_callback)
	control_flag_sub = rospy.Subscriber("/control_flag", Int8, control_flag_callback)
	
	dynamic_reconfigure_server = Server(paramsConfig, reconfigCB)

	out_l_center_target = rospy.get_param('~out_l_center_target', 0.26)
	out_l_vel_z_P = rospy.get_param('~out_l_vel_z_P', 0.01) 
	out_l_vel_z_D = rospy.get_param('~out_l_vel_z_D', 0.001) 
	out_r_center_target = rospy.get_param('~out_r_center_target', 0.75) 
	out_r_vel_z_P = rospy.get_param('~out_r_vel_z_P', 0.01) 
	out_r_vel_z_D = rospy.get_param('~out_r_vel_z_D', 0.001) 
	in_l_center_target = rospy.get_param('~in_l_center_target', 0.28) 
	in_l_vel_y_P = rospy.get_param('~in_l_vel_y_P', 0.001) 
	in_l_vel_y_D = rospy.get_param('~in_l_vel_y_D', 0.003) 
	in_l_vel_z_P = rospy.get_param('~in_l_vel_z_P', 0.006) 
	in_l_vel_z_D = rospy.get_param('~in_l_vel_z_D', 0.001) 
	in_r_center_target = rospy.get_param('~in_r_center_target', 0.75) 
	in_r_vel_y_P = rospy.get_param('~in_r_vel_y_P', 0.001) 
	in_r_vel_y_D = rospy.get_param('~in_r_vel_y_D', 0.003) 
	in_r_vel_z_P = rospy.get_param('~in_r_vel_z_P', 0.006) 
	in_r_vel_z_D = rospy.get_param('~in_r_vel_z_D', 0.001) 

	left_stop_xmin = rospy.get_param('~left_stop_xmin', 111)
	left_stop_ymin = rospy.get_param('~left_stop_ymin', 278)
	left_stop_xmax = rospy.get_param('~left_stop_xmax', 193)
	left_stop_ymax = rospy.get_param('~left_stop_ymax', 333)
	right_stop_xmin = rospy.get_param('~right_stop_xmin', 470)
	right_stop_ymin = rospy.get_param('~right_stop_ymin', 282)
	right_stop_xmax = rospy.get_param('~right_stop_xmax', 556)
	right_stop_ymax = rospy.get_param('~right_stop_ymax', 336)


	road_con_par_rig_min = rospy.get_param('~road_con_par_rig_min', -100)
	road_con_par_rig_max = rospy.get_param('~road_con_par_rig_max', -80)
	road_con_par_left_min = rospy.get_param('~road_con_par_left_min', 100)
	road_con_par_left_max = rospy.get_param('~road_con_par_left_max', 120)
	
	rate = rospy.Rate(100)
	
	while not rospy.is_shutdown():
		#print(out_l_center_target)
		if control_flag == 1:
			if currentState == 0:		#init    
				a = line_judgment()
				msg = a
				line_pub.publish(msg)
				#print(a)
				if a > 0:
					temp1 = temp1 +1
					temp2 = 0
					if temp1 > 50:
						side = 2
						currentState = 1
						last_currentState = currentState
						last_side = side
						#pub  out_ri
						ctrldata_pub = ctrl_data(currentState, side)
						ctrl_pub.publish(ctrldata_pub)
						print("out_ri")
						temp1 = 0
						side = 0
				elif a < 0:
					temp2 = temp2 +1
					temp1 = 0
					if temp2 > 50:
						side = 1
						currentState = 1
						last_currentState = currentState
						last_side = side
						#pub  out_le
						ctrldata_pub = ctrl_data(currentState, side)
						ctrl_pub.publish(ctrldata_pub)
						print("out_le")
						temp2 = 0
						side = 0
				else:
					temp1 = 0
					temp2 = 0
					side = 0
					currentState = 0
					print("No line init!!")
			
			elif currentState == 1:		#outside
				if i % 150 == 0:
					print("outside")
				#-----bus_station and crossing------
				if (crossing_flag == 0 or bus_flag == 0) and count1 != -1:
					if count1 > i :
						b = i + 50000
					else: 
						b = i
					if b - count1 > 300:
						decelerate = 15
						decelerate_pub.publish(decelerate)
						count1 = -1

				elif crossing_flag == 1 or bus_flag == 1:
					decelerate = 5
					decelerate_pub.publish(decelerate)
					crossing_flag = 0
					bus_flag = 0
					count1 = i

				#-----stop------
				if stop_flag == 0 and count2 != -1:
					#print("stop_flag == 0")
					if count2 > i :
						c = i + 50000
					else: 
						c = i
					if c - count2 > 200:
						ctrl_pub.publish(ctrldata_pub)
						count2 = -1
						#stop_count = 0

				elif stop_flag == 1:
					# stop_count = stop_count + stop_flag
					# stop_flag = 0
					# if stop_count > 5:
					#print("stop_flag == 1")
					car_stop = CtrlData()
					ctrl_pub.publish(car_stop)
					if stop_x > 320 :
						erro_x = stop_x - ((right_stop_xmin +right_stop_xmax)/2)
						erro_y = stop_y - ((right_stop_ymin +right_stop_ymax)/2)
					else :
						erro_x = stop_x - ((left_stop_xmin +left_stop_xmax)/2)
						erro_y = stop_y - ((left_stop_ymin +left_stop_ymax)/2)
					d_erro_x = erro_x - last_erro_x
					d_erro_y = erro_y - last_erro_y
					carmove = Twist()
					carmove.linear.x = -float(erro_y)*0.0005 - float(d_erro_y)*0.0
					carmove.linear.y = -float(erro_x)*0.0008 - float(d_erro_x)*0.0
					last_erro_x=erro_x
					last_erro_y=erro_y
					if -0.01 < carmove.linear.x < 0.005 :
						carmove.linear.x = 0.0
					if -0.01 < carmove.linear.y < 0.005:
						carmove.linear.y = 0.0
					if carmove.linear.x == 0 and carmove.linear.y == 0 :
						d_erro_x = 0
						d_erro_y = 0
					cmdvel_pub.publish(carmove)
					stop_flag = 0
					count2 = i

				#-----turn------
				# if side_flag == 1:
				# 	temp1 = temp1 +1
				# 	temp2 = 0
				# 	side_flag = 0
				# 	if temp1 > 10:
				# 		side = 1
				# 		temp1 = 0
				# elif side_flag == 2:
				# 	temp2 = temp2 +1
				# 	temp1 = 0
				# 	side_flag = 0
				# 	if temp2 > 10:
				# 		side = 2
				# 		temp2 = 0
				if side_flag == 1:
					side = 1
					side_flag = 0
				elif side_flag == 2:
					side = 2
					side_flag = 0

				if side == 1:
					# if -2.785 < pos_angleX < -2.385:	#A2
					if -1.9 < pos_angleX < -1.6:	#A1
						currentState = 2
						last_currentState = currentState
						last_side = side
						#pub  in_le
						ctrldata_pub = ctrl_data(currentState, side)
						ctrl_pub.publish(ctrldata_pub)
						print("in_le")
						side = 0
						side_flag = 0
						count1 = -1
						count2 = -1
						t_junction = False
				elif side == 2:
					# if 0.815 < pos_angleX < 1.215:	#A2
					if 1.6 < pos_angleX < 1.9:	#A1
						# temp2 = temp2 +1
						# temp1 = 0
						# if temp2 > 1:
						currentState = 2
						last_currentState = currentState
						last_side = side
						#pub  in_ri
						ctrldata_pub = ctrl_data(currentState, side)
						ctrl_pub.publish(ctrldata_pub)
						print("in_ri")
						side = 0
						side_flag = 0
						# temp2 = 0
						count1 = -1
						count2 = -1
						t_junction = False

			elif currentState == 2:		#inside
				if i % 150 == 0:
					print("inside")
				#-----road_construction------
				if road_construction_flag == 1:
					car_stop = CtrlData()
					ctrl_pub.publish(car_stop)
					a = line_judgment()
					print(a)
					ymove= Twist()
					if ctrldata_pub.mask_x_left > 0:
						if road_con_par_left_min < a < road_con_par_left_max:    #-100 < a < -80:
							currentState = 1
							last_currentState = currentState
							last_side = 1
							ctrldata_pub = ctrl_data(currentState, 1)
							ctrl_pub.publish(ctrldata_pub)
							side = 0
							road_construction_flag = 0
						else:
							ymove.linear.y = 0.1
							cmdvel_pub.publish(ymove)
					else:
						if  road_con_par_rig_min < a < road_con_par_rig_max:  #100< a < 120:
							currentState = 1
							last_currentState = currentState
							last_side = 2
							ctrldata_pub = ctrl_data(currentState, 2)
							ctrl_pub.publish(ctrldata_pub)
							side = 0
							road_construction_flag = 0
						else:
							ymove.linear.y = -0.1
							cmdvel_pub.publish(ymove)

				#-----turn------
				# if side_flag == 1:
				# 	temp1 = temp1 +1
				# 	temp2 = 0
				# 	side_flag = 0
				# 	if temp1 > 10:
				# 		side = 1
				# 		temp1 = 0
				# elif side_flag == 2:
				# 	temp2 = temp2 +1
				# 	temp1 = 0
				# 	side_flag = 0
				# 	if temp2 > 10:
				# 		side = 2
				# 		temp2 = 0
				
				if side_flag == 1:
					side = 1
					side_flag = 0
				elif side_flag == 2:
					side = 2
					side_flag = 0
				# if side != 0:
				# 	t_junction = T_junction()

					# print(t_junction)
					# a = T_junction()
					# print(a)
					# if a == True:
					# 	temp3 = temp3 + 1
					# 	if temp3 > 3:
					# 		t_junction = True
					# 		temp3 = 0
					# else:
					# 	t_junction = False
				
				if side == 1:
					# if -2.785 < pos_angleX < -2.385:	#A2
					if -1.85 < pos_angleX < -1.6:	#A1
						currentState = 2
						last_currentState = currentState
						last_side = side
						#pub  in_le
						ctrldata_pub = ctrl_data(currentState, side)
						ctrl_pub.publish(ctrldata_pub)
						print("in_le")
						side = 0
						side_flag = 0
						time.sleep(2)
						#print(t_junction)
						temp4 = 0
						while(t_junction == False and temp4 < 10):
							t_junction = T_junction()
							print(t_junction)
							temp4 = temp4 +1
						if(t_junction == True):
							time.sleep(7)
							currentState = 1
							side = 2
							last_currentState = currentState
							last_side = side
							#pub  out_ri
							ctrldata_pub = ctrl_data(currentState, side)
							ctrl_pub.publish(ctrldata_pub)
							print("out_ri")
							side = 0
							side_flag = 0

				elif side == 2:
					# if 0.815 < pos_angleX < 1.215:   #A2
					if 1.6 < pos_angleX < 1.85:     #A1
						currentState = 2
						last_currentState = currentState
						last_side = side
						#pub  in_ri
						ctrldata_pub = ctrl_data(currentState, side)
						ctrl_pub.publish(ctrldata_pub)
						print("in_ri")
						side = 0
						side_flag = 0
						time.sleep(2)
						temp4 = 0
						while(t_junction == False and temp4 < 10):
							t_junction = T_junction()
							print(t_junction)
							temp4 = temp4 +1
						if(t_junction == True):
							time.sleep(6)
							currentState = 1
							side = 1
							last_currentState = currentState
							last_side = side
							#pub  out_le
							ctrldata_pub = ctrl_data(currentState, side)
							ctrl_pub.publish(ctrldata_pub)
							print("out_le")
							side = 0
							side_flag = 0

			i = i + 1
			if i > 50000:
				i = 0
			
		else :
			print("waiting for init")
		rate.sleep()
	#rospy.spin()


if __name__ == '__main__':
    try:
    	control_drive()
    except rospy.ROSInterruptException:
    	pass
