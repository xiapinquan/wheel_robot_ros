#!/usr/bin/env python
# coding=utf-8
import rospy 

import math
import PyKDL
from geometry_msgs.msg import PoseWithCovarianceStamped


class PoseSetter(rospy.SubscribeListener):
    def __init__(self, pose, stamp, publish_time):
        self.pose = pose
        self.stamp = stamp
        self.publish_time = publish_time

        global multi_mode
        global slave_robot2
        global slave_robot3

        global position_x
        global position_y

        multi_mode = rospy.get_param('~multi_mode')
        slave_robot2 = rospy.get_param('~slave_robot2')
        slave_robot3 = rospy.get_param('~slave_robot3')

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        p = PoseWithCovarianceStamped()
        #设定初始位姿距原点的距离，单位为米
        slave1_bias_x = 0.6
        slave1_bias_y = 0
        #left
        row1_bias_x = 0
        row1_bias_y = -0.8

        slave2_bias_x = 1.4
        slave2_bias_y = 0
        #right
        row2_bias_x = 0
        row2_bias_y = 0.8
        #纵向队形 slave模式下1号从车的初始位姿
        if (not multi_mode) and slave_robot2 :
            position_x = self.pose[0] - slave1_bias_x
            position_y = self.pose[1] - slave1_bias_y
        #纵向队形 slave模式下2号从车的初始位姿
        elif (not multi_mode) and slave_robot3 :
            position_x = self.pose[0] - slave2_bias_x
            position_y = self.pose[1] - slave2_bias_y
        #横向队形 row模式下1号从车的初始位姿
        elif (multi_mode) and slave_robot2 :
            position_x = self.pose[0] - row1_bias_x
            position_y = self.pose[1] - row1_bias_y
        #横向队形 row模式下2号从车的初始位姿
        elif (multi_mode) and slave_robot3 :
            position_x = self.pose[0] - row2_bias_x
            position_y = self.pose[1] - row2_bias_y
        else:
            position_x = self.pose[0]
            position_y = self.pose[1]

        p.pose.pose.position.x = position_x
        p.pose.pose.position.y = position_y
        (p.pose.pose.orientation.x,
         p.pose.pose.orientation.y,
         p.pose.pose.orientation.z,
         p.pose.pose.orientation.w) = PyKDL.Rotation.RPY(0, 0, self.pose[2]).GetQuaternion()
        p.pose.covariance[6*0+0] = 0.5 * 0.5
        p.pose.covariance[6*1+1] = 0.5 * 0.5
        p.pose.covariance[6*3+3] = math.pi/12.0 * math.pi/12.0
        # wait for the desired publish time
        while rospy.get_rostime() < self.publish_time:
            rospy.sleep(0.01)
        peer_publish(p)


if __name__ == '__main__':
    pose = map(float, rospy.myargv()[1:4])
    t_stamp = rospy.Time()
    t_publish = rospy.Time()
    if len(rospy.myargv()) > 4:
        t_stamp = rospy.Time.from_sec(float(rospy.myargv()[4]))
    if len(rospy.myargv()) > 5:
        t_publish = rospy.Time.from_sec(float(rospy.myargv()[5]))
    rospy.init_node('pose_setter', anonymous=True)
    rospy.loginfo("Going to publish pose {} with stamp {} at {}".format(pose, t_stamp.to_sec(), t_publish.to_sec()))
    pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, PoseSetter(pose, stamp=t_stamp, publish_time=t_publish), queue_size=1)
    rospy.spin()
