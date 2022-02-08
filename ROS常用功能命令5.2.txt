1.键盘控制
//需要先开启初始化节点（仅在单独开启键盘控制时需要开启 运行功能时已包括初始化节点 不需要重复开启）
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
//开启键盘控制节点
roslaunch wheeltec_robot_rc keyboard_teleop.launch

2.巡线(雷达避障)
roslaunch simple_follower line_follower.launch 

3.雷达跟随。
roslaunch simple_follower laser_follower.launch 

4.视觉跟踪。
roslaunch simple_follower visual_follower.launch 

5.2D建图、2D导航。
roslaunch turn_on_wheeltec_robot mapping.launch 
roslaunch turn_on_wheeltec_robot navigation.launch 
一键保存地图(WHEELTEC.pgm、WHEELTEC.yaml)
roslaunch turn_on_wheeltec_robot map_saver.launch 

6.3D建图、3D导航。
roslaunch turn_on_wheeltec_robot 3d_mapping.launch 
roslaunch turn_on_wheeltec_robot 3d_navigation.launch 

7.纯视觉建图导航
roslaunch turn_on_wheeltec_robot pure3d_mapping.launch
roslaunch turn_on_wheeltec_robot pure3d_navigation.launch

8.语音控制
//开启底层、导航、雷达扫描节点
roslaunch xf_mic_asr_offline base.launch
//开启麦克风阵列初始化节点
roslaunch xf_mic_asr_offline mic_init.launch

9.KCF跟随
roslaunch kcf_track kcf_tracker.launch

10.自主建图 详见自主建图功能教程
//启动rrt_slam.launch文件：
roslaunch turn_on_wheeltec_robot rrt_slam.launch
//打开rviz，点击左下方“add”→“by topic”增加配置插件：
clicked_point	显示随机数的范围点和起点
detected_points	检测到的边界点
frontiers		滤波器接收到的边界点，数据同上
centroids		滤波后的有效边界点
global_detector_shapes	全局树
local_detector_shapes	本地树
//用rviz的publish point工具，按顺时针或者逆时针设置4个生长树的边界点，以及一个生长树起点（起点尽量靠近机器人起点），设置完成后机器人便依据生长树去探索地图。

11.WHEELTEC APP图传、建图与导航
//用户手机连接小车wifi，打开APP即可使用。该APP可控制小车移动、保存地图、查看摄像头画面，详见WHEELTEC APP功能教程
//图传，需要手动打开RGB摄像头节点：roslaunch usb_cam usb_cam-test.launch
APP端可以实时观看到摄像头画面
//建图，需要手动打开建图节点：roslaunch turn_on_wheeltec_robot mapping.launch 
APP端可以查看建图效果并保存，同时可以控制小车移动
//导航，需要手动打开导航节点：roslaunch turn_on_wheeltec_robot navigation.launch 
APP端可以控制小车移动

12.多机编队
//首先所有小车必须在同一个网络(wifi)下，然后修改.bashrc文件设置主从机，详见多机编队教程
//使用ssh命令远程登录之后，在分割终端中的左上角点击[广播到所有]，使用命令进行时间同步
sudo date -s "2021-01-30 08:48:00"
//主机开启导航节点
roslaunch turn_on_wheeltec_robot navigation.launch 
//从机1开启从动初始化节点
roslaunch wheeltec_multi wheeltec_slave.launch
//从机2开启从动初始化节点 (2个从机需要使用高性能路由器提供wifi)
roslaunch wheeltec_multi wheeltec_slave.launch
//主机开启控制从机节点
roslaunch wheeltec_multi robot_tf.launch

13.WEB浏览器显示摄像头
主机：roslaunch usb_cam usb_cam-test.launch
          rosrun web_video_server web_video_server
主机网页查看：http://localhost:8080/ (发出热点的为主机)
客户机网页查看：http://192.168.0.100:8080 (连接热点的为客户机)
【注】建议使用谷歌浏览器，经测试360极速浏览器、IE浏览器无法打开图像

14.物体识别(不支持树莓派4B 2GB)
物体识别：roslaunch ros_detection ros_tensorflow_classify.launch
查看摄像头实时画面：rqt_image_view(选择"/camera/rgb/image_raw/compressed"话题)

15.AR标签识别
roslaunch turn_on_wheeltec_robot ar_label.launch
创建一个二维码，边长为5，内容为0
rosrun ar_track_alvar createMarker -s 5 0
AR标签跟随
roslaunch simple_follower ar_follower.launch
   
16.2.4G无线手柄控制ROS端
//需要先开启初始化节点
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
//开启无线手柄控制节点
roslaunch wheeltec_joy joy_control.launch

17.深度学习
//开启深度学习节点
roslaunch darknet_ros darknet_ros.launch
//开启手势识别动作节点
roslaunch wheeltec_yolo_action gesture.launch
//开启沙盘运动节点
roslaunch wheeltec_yolo_action dp_drive.launch

18.Gazebo建图导航仿真
//2D gazebo建图
roslaunch wheeltec_gazebo_function mapping.launch
//键盘控制
roslaunch wheeltec_gazebo_function keyboard_teleop.launch
//保存地图
一键保存地图(WHEELTEC.pgm、WHEELTEC.yaml)
roslaunch wheeltec_gazebo_function map_saver.launch
//2D gazebo导航
roslaunch wheeltec_gazebo_function navigation.launch
------------------------------------------
其它常用命令

递归修改当前(终端)文件夹下文件修改时间：
find ./* -exec touch {} \;

在工作空间下运行，安装ROS功能包全部依赖：
rosdep install --from-paths src --ignore-src -r -y

指定功能包编译：
catkin_make -DCATKIN_WHITELIST_PACKAGES="功能包名"
解除指定功能包编译：
catkin_make -DCATKIN_WHITELIST_PACKAGES=""

使用豆瓣源进行pip安装(网速会快很多)：
pip install -i https://pypi.doubanio.com/simple/ python包名

ssh登录：
ssh -Y wheeltec@192.168.0.100

nfs挂载:
sudo mount -t nfs 192.168.0.100:/home/wheeltec/wheeltec_robot /mnt
nfs解除挂载:
sudo umount -t nfs 192.168.0.100:/home/wheeltec/wheeltec_robot /mnt

打开地图路径：
cd /home/wheeltec/wheeltec_robot/src/turn_on_wheeltec_robot/map
手动保存地图：
rosrun map_server map_saver -f 20200714

打开RGB摄像头
roslaunch usb_cam usb_cam-test.launch
rqt_image_view

打开深度摄像头
roslaunch astra_camera astrapro.launch 
rqt_image_view

查看节点与话题关系
rqt_graph

生成TF树pdf
rosrun tf view_frames
查看TF树
rosrun rqt_tf_tree rqt_tf_tree
