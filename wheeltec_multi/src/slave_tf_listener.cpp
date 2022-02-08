/**************************************************************************
作者：pjf
功能：多机编队
**************************************************************************/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"

double angular_z = 0;    //通过计算得出的slave与master的角度
double linear_x = 0;    //通过计算得出的slave与master的距离

double last_angular_z = 0;  
double last_linear_x = 0;
double ddyz = 0;
double ddyx = 0;
double plusandminus = 1.3;
double max_vel = 0.4;
double normal_vel = 0.2;

// 增益系数调节
double Scalex_P = 0; 
double Scalex_I = 0;
double Scalez_P =0;
double Scalez_I =0;
// 编队队形标志位
double roworleng = 1;

// tf变换相关参数
std::string base_frame;
std::string base_to_row;
std::string base_to_slave;
std::string tf_prefix_;

int main(int argc, char** argv){
  ros::init(argc, argv, "wheeltec_multi");

  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  ros::Publisher slave_vel = node.advertise<geometry_msgs::Twist>("cmd_vel_ori", 10); //发布原始数据

  private_nh.param<double>("Scalex_P", Scalex_P, 0);  
  private_nh.param<double>("Scalex_I", Scalex_I, 0);   
  private_nh.param<double>("Scalez_I", Scalez_I, 0);  
  private_nh.param<double>("Scalez_P", Scalez_P, 0);  
  private_nh.param<double>("roworleng", roworleng,1);

  private_nh.param<double>("max_vel", max_vel,0.4);  
  private_nh.param<double>("normal_vel", normal_vel,0.2);  

  private_nh.param<double>("plusandminus", plusandminus,1.3);  //判断为从车在主车前方的阈值

  private_nh.param<std::string>("base_frame", base_frame, "base_footprint");
  private_nh.param<std::string>("base_to_row", base_to_row, "row2");
  private_nh.param<std::string>("base_to_slave", base_to_slave, "slave2");

 // 使用tf_prefix参数将frame_name解析为frame_id
  tf_prefix_ = tf::getPrefixParam(private_nh); 
  base_frame = tf::resolve(tf_prefix_, base_frame);
  base_to_row = tf::resolve(tf_prefix_, base_to_row);
  base_to_slave = tf::resolve(tf_prefix_, base_to_slave);

  tf::TransformListener listener;
  geometry_msgs::Twist vel_msg;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transformSM;
    //transformSM.header.stamp = ros::Time(0);
    //由 Slave 到 master 做tf变换
    try{  
      if(roworleng)
      {
        listener.waitForTransform(base_frame, base_to_row, ros::Time(0), ros::Duration(3.0));

        listener.lookupTransform(base_frame, base_to_row,
                               ros::Time(0), transformSM);
          printf("now the formation is row\n");
      }
    else
      {
        listener.waitForTransform(base_frame, base_to_slave, ros::Time(0), ros::Duration(3.0));

        listener.lookupTransform(base_frame,  base_to_slave, ros::Time(0), transformSM);
          printf("now the formation is slave\n");
      }
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue; 
    } 

    angular_z = atan2((transformSM.getOrigin().y()),
                                   (transformSM.getOrigin().x()));
    linear_x = sqrt(pow((transformSM.getOrigin().x()), 2) +
                                  pow((transformSM.getOrigin().y()), 2));
    printf("The distance bias= %f\n",linear_x);    // 输出误差数据信息
    printf("The angle bias= %f\n",angular_z);
    if(fabs(angular_z)> plusandminus) //当角度的绝对值大于这个值时判断为从车在主车前方
      linear_x = -linear_x;

    ddyx = linear_x - last_linear_x ; 
    ddyz = angular_z - last_angular_z ;

    // 避免静态误差
    if (fabs(linear_x) > 0.1) //针对大车改动
    {
      vel_msg.linear.x = Scalex_I * linear_x + Scalex_P * ddyx;
      vel_msg.angular.z = Scalez_I * angular_z + Scalez_P * ddyz;

    if (fabs(vel_msg.linear.x) > max_vel) //速度限幅
      vel_msg.linear.x = normal_vel;
    if (fabs(vel_msg.angular.z) > max_vel)
      vel_msg.angular.z = normal_vel;

    if((fabs(angular_z)>1.3&& fabs(linear_x) < 0.25)||(fabs(angular_z)>1.4&&fabs(vel_msg.linear.x<0.08)&&fabs(vel_msg.linear.x>0)))//阿克曼小车结构问题，避免转弯后到达目标点后依旧会调整自身姿态
      {
        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        vel_msg.angular.z = 0;
        printf("Goal reached!\n");

      }
      last_linear_x = linear_x;
      last_angular_z = angular_z;
    }
    else 
    {
      vel_msg.linear.x = 0;
      vel_msg.linear.y = 0;
      vel_msg.angular.z = 0;
    }

    printf("vx= %f\n",vel_msg.linear.x);
    printf("vz= %f\n",vel_msg.angular.z);

    slave_vel.publish(vel_msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};
