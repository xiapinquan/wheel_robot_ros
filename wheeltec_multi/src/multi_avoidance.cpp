/**************************************************************************
功能：避障
**************************************************************************/
#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <wheeltec_multi/avoid.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>


using namespace std;
 
geometry_msgs::Twist cmd_vel_msg;    //速度控制信息数据
geometry_msgs::Twist cmd_vel_avoid;    //速度控制信息数据
geometry_msgs::Twist cmd_vel_data;    //速度控制信息数据

float distance1;    //障碍物距离
float dis_angleX;    //障碍物方向


/**************************************************************************
函数功能：sub回调函数
入口参数：  laserTracker.py
返回  值：无
**************************************************************************/
void current_position_Callback(const wheeltec_multi::avoid& msg)	
{
	distance1 = msg.distance;
	dis_angleX = msg.angleX;
}

/**************************************************************************
函数功能：底盘运动sub回调函数（原始数据）
入口参数：cmd_msg  command_recognition.cpp
返回  值：无
**************************************************************************/
void cmd_vel_ori_Callback(const geometry_msgs::Twist& msg)
{
	cmd_vel_msg.linear.x = msg.linear.x;
	cmd_vel_msg.angular.z = msg.angular.z;

	cmd_vel_data.linear.x = msg.linear.x;
	cmd_vel_data.angular.z = msg.angular.z;
}

/**************************************************************************
函数功能：判断障碍物距离是否小于0.75米
入口参数：无
返回  值：1或0
**************************************************************************/
int distance_judgment(void)
{
	//int a;
	if(distance1<=0.4) 
	{
		ROS_INFO("distance1 less then 0.4 ");
		printf("distance1= %f\n",distance1);

		return 1;
	}
	else
		return 0;
	
}
 
/**************************************************************************
函数功能：判断障碍物方向是否在小车运动趋势方向上
入口参数：无
返回  值：1或0
**************************************************************************/
int dis_angleX_judgment(void)
{
	if(cmd_vel_msg.linear.x > 0 && (dis_angleX >  2.335 || dis_angleX < -2.335))
		{
		ROS_INFO("dis_angleX_judgment");

		ROS_INFO("dis_angleX_judgment");
		int temp_count = 0;    //计数变量

		if(((dis_angleX >  2.75 && dis_angleX < 3.14)) || ((dis_angleX >  -3.14 && dis_angleX < -2.75)) )// 障碍在机器人正前方
			{
			temp_count++; 
			if(temp_count > 5)
			{
				cmd_vel_avoid.linear.x  =  -0.3;
				cmd_vel_avoid.linear.y  =  0;
				cmd_vel_avoid.angular.z  =  0.3;
				temp_count = 0;
			}
			else{
				cmd_vel_avoid.linear.x  =  -0.25; // 后退转左
				cmd_vel_avoid.linear.y  =  0.0;
				cmd_vel_avoid.angular.z = 0.3;

			}

			ROS_INFO("turn left ");

			}
		else if(dis_angleX >  2 && dis_angleX < 2.75) // 障碍在机器人右前方
			{
			cmd_vel_avoid.linear.x  =  0.12; // 转左
			cmd_vel_avoid.linear.y  =  0.0;
			cmd_vel_avoid.angular.z = 0.25;
			ROS_INFO("turn left ");
			}
		else if(dis_angleX >  -2.75 && dis_angleX < -2) // 障碍在机器人左前方
			{
			cmd_vel_avoid.linear.x  = 0.12;   // 转右
			cmd_vel_avoid.linear.y  = 0.0;
			cmd_vel_avoid.angular.z = -0.25;
			ROS_INFO("turn right ");
			}
		else
			{
			cmd_vel_avoid.linear.x  = 0.12;  // 其他情况
			cmd_vel_avoid.linear.y  = 0.0;
			cmd_vel_avoid.angular.z = 0.25;
			ROS_INFO("turn right ");
			}
		return 1;
		}
	else if(cmd_vel_msg.linear.x > 0 && (dis_angleX >  -2 && dis_angleX < -1.67))  // 障碍在机器人左侧
		{
			cmd_vel_avoid.linear.x  = 0.12;  // 转右
			cmd_vel_avoid.linear.y  = 0.0;
			cmd_vel_avoid.angular.z = -0.25;
			ROS_INFO("turn right ");
		return 1;
		}
	else if(cmd_vel_msg.linear.x > 0  && (dis_angleX > 1.67 && dis_angleX < 2)) // 障碍在机器人右侧
		{
			cmd_vel_avoid.linear.x  =  0.12;   // 转左
			cmd_vel_avoid.linear.y  =  0.0;
			cmd_vel_avoid.angular.z = 0.25;
			ROS_INFO("turn left ");
		return 1;
		}
		
	else 
		return 0;
		
}


/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/
int main(int argc, char** argv)
{
	int temp_count = 0;    //计数变量
	ros::init(argc, argv, "avoidance");    //初始化ROS节点

	ros::NodeHandle node;    //创建句柄

	/***创建底盘速度控制话题发布者***/
	ros::Publisher cmd_vel_Pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	/***创建底盘运动话题订阅者***/
	ros::Subscriber vel_sub = node.subscribe("cmd_vel_ori", 1, cmd_vel_ori_Callback);

  	/***创建障碍物方位话题订阅者***/
	ros::Subscriber current_position_sub = node.subscribe("object_tracker/current_position", 1, current_position_Callback);

	
	double rate2 = 10;    //频率10Hz
	ros::Rate loopRate2(rate2);

 
	while(ros::ok())
	{
		ros::spinOnce();
			
		if(distance_judgment() && dis_angleX_judgment())    //判断障碍物的距离和方向
		{
			temp_count++;
			if(temp_count > 5)    //连续计数5️次停止运动防止碰撞，避免雷达有噪点
			{
				ROS_INFO("Obstacle detected, implement avoidance action.");
   				if (fabs(cmd_vel_avoid.linear.x)>0.05) // 排除噪点
   			 	{
					cmd_vel_msg = cmd_vel_avoid;
   				 }
  			 	else 
 		   		{
					cmd_vel_avoid.linear.x  =  0;  
					cmd_vel_avoid.linear.y  =  0;
					cmd_vel_avoid.angular.z =  0;
   			 	}
				cmd_vel_Pub.publish(cmd_vel_msg);    //将速度指令发送给机器人
			}
		}
		else
		{
			ROS_INFO("Obstacle clear.");
			if (fabs(cmd_vel_data.linear.x)>0.05)   //排除噪点
   			{
				cmd_vel_msg = cmd_vel_data;
   			}
  			else 
 		    {
				cmd_vel_data.linear.x  =  0;  
				cmd_vel_data.linear.y  =  0;
				cmd_vel_data.angular.z =  0;
   			}
			temp_count = 0;    //排除雷达噪点
			cmd_vel_msg = cmd_vel_data;
			cmd_vel_Pub.publish(cmd_vel_msg);    //将速度指令发送给机器人
		}
		ros::spinOnce();
		loopRate2.sleep();
	} 

	return 0;
}
