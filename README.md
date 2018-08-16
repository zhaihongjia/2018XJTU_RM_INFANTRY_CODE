# 2018XJTU_RM_INFANTRY_CODE
＃－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－

西安交通大学RoboMaster笃行战队步兵机器人视觉部分代码，主要包括：大小能量机关，装甲自瞄以及补给站WiFi通讯

＃－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－

整个代码是基于ROS实现的，主要包括三个package:

opencvtest3:大小能量机关和自瞄节点，补给站消息节点

serial_common:串口节点设置

usb_cam:摄像头节点设置

models和my_code_data 包含一些大小能量机关中识别的Caffe模型，网络定义以及摄像头参数

＃－－－－－－－－－－－－－－－－－－－－－

依赖环境：

cmake 3.5.1

opencv3.3.1

ubuntu16.04

caffe

ROS kinetic

v4l2

＃－－－－－－－－－－－－－－－－－－－－－


＃－－－－－－－编译运行方式：－－－－－－－

cd 

mkdir catkin_ws

cd catkin_ws/

mkdir src

将opencvtest3 serial_common usb_cam 节点包　copy 至src 目录下；修改opencvtest3/src/bubing.cpp中网络文件的路径（即model and my_code_data 中文件路径）

在catkin_ws目录下执行：

catkin_make

编译通过即可

＃－－－－－－－－－－－－－－－－－－－－－

如有相关疑问，可以咨询相关人员,欢迎和大家一起讨论交流，微信联系方式如下：

大小能量机关：翟宏佳：zhj395019589
装甲自瞄：陈宇航：niruochengfengcyh
补给站WiFi：陈凯：ckxajd
