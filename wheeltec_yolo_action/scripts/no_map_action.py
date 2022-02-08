#!/usr/bin/env python
# coding=utf-8

import rospy
import threading
import time
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes

boundingbox_id = -1
temp0 = 0
temp1 = 0
temp2 = 0
temp3 = 0
temp4 = 0
temp5 = 0
count = 2


def thread_job():
	rospy.spin()

def side_flag_callback(msg):
	global boundingbox_id
	global temp0
	global temp1
	global temp2
	global temp3
	global temp4
	global temp5
	global count
	# for boxes in msg.bounding_boxes:
	# 	if boxes.probability > 0.8:
	# 		boundingbox_id = boxes.id
	# 		print(boxes.Class)
	# print("--------------")
	for boxes in msg.bounding_boxes:
		if boxes.probability > 0.7 and boxes.id == 0:
			temp0 = temp0 + 1
			temp1 = 0
			temp2 = 0
			temp3 = 0
			temp4 = 0
			temp5 = 0
			if temp0 > count:
				boundingbox_id = boxes.id
				temp0 = 0
		elif boxes.probability > 0.7 and boxes.id == 1:
			temp1 = temp1 + 1
			temp0 = 0
			temp2 = 0
			temp3 = 0
			temp4 = 0
			temp5 = 0
			if temp1 > count:
				boundingbox_id = boxes.id
				temp1 = 0
		elif boxes.probability > 0.7 and boxes.id == 2:
			temp2 = temp2 + 1
			temp0 = 0
			temp1 = 0
			temp3 = 0
			temp4 = 0
			temp5 = 0
			if temp2 > count:
				boundingbox_id = boxes.id
				temp2 = 0
		elif boxes.probability > 0.7 and boxes.id == 3:
			temp3 = temp3 + 1
			temp0 = 0
			temp1 = 0
			temp2 = 0
			temp4 = 0
			temp5 = 0
			if temp3 > count:
				boundingbox_id = boxes.id
				temp3 = 0
		elif boxes.probability > 0.7 and boxes.id == 4:
			temp4 = temp4 + 1
			temp0 = 0
			temp1 = 0
			temp2 = 0
			temp3 = 0
			temp5 = 0
			if temp4 > count:
				boundingbox_id = boxes.id
				temp4 = 0 
		elif boxes.probability > 0.7 and boxes.id == 5:
			temp5 = temp5 + 1
			temp0 = 0
			temp1 = 0
			temp2 = 0
			temp3 = 0
			temp4 = 0
			if temp5 > count:
				boundingbox_id = boxes.id
				temp5 = 0

def control_action():
	global boundingbox_id

	rospy.init_node("control_drive")
	add_thread = threading.Thread(target = thread_job)
	add_thread.start

	cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	side_flag_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, side_flag_callback)
	car_mode =rospy.get_param("/if_akm_yes_or_no","no")

	cmd_msg = Twist()
	rate = rospy.Rate(100)
	print("start node!")
	while not rospy.is_shutdown():
		#print(car_mode)
		if boundingbox_id == 0:
			if car_mode == "yes":
				cmd_msg.linear.x = 0.2
				cmd_msg.angular.z = 1
				cmdvel_pub.publish(cmd_msg)
				time.sleep(24)
				cmd_msg.linear.x = 0
				cmd_msg.angular.z = 0
				cmdvel_pub.publish(cmd_msg)
			else:
				cmd_msg.angular.z = 1
				cmdvel_pub.publish(cmd_msg)
				time.sleep(6.7)
				cmd_msg.angular.z = 0
				cmdvel_pub.publish(cmd_msg)
			boundingbox_id = -1
		elif boundingbox_id == 5:
			if car_mode == "yes":
				cmd_msg.linear.x = 0.2
				cmd_msg.angular.z = -1
				cmdvel_pub.publish(cmd_msg)
				time.sleep(24)
				cmd_msg.linear.x = 0
				cmd_msg.angular.z = 0
				cmdvel_pub.publish(cmd_msg)
			else:
				cmd_msg.angular.z = -1
				cmdvel_pub.publish(cmd_msg)
				time.sleep(6.7)
				cmd_msg.angular.z = 0
				cmdvel_pub.publish(cmd_msg)
			boundingbox_id = -1 
		elif boundingbox_id == 2:
			cmd_msg.linear.x = 0.2
			cmdvel_pub.publish(cmd_msg)
			time.sleep(3)
			cmd_msg.linear.x = -0.2
			cmdvel_pub.publish(cmd_msg)
			time.sleep(3)
			cmd_msg.linear.x = 0
			cmdvel_pub.publish(cmd_msg)
			boundingbox_id = -1 
		elif boundingbox_id == 3:
			cmd_msg.linear.x = -0.2
			cmdvel_pub.publish(cmd_msg)
			time.sleep(3)
			cmd_msg.linear.x = 0.2
			cmdvel_pub.publish(cmd_msg)
			time.sleep(3)
			cmd_msg.linear.x = 0
			cmdvel_pub.publish(cmd_msg)
			boundingbox_id = -1 





if __name__ == '__main__':
    try:
    	control_action()
    except rospy.ROSInterruptException:
    	pass
