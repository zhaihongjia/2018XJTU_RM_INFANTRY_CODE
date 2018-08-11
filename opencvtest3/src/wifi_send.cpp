#include <ros/ros.h>
#include<sensor_msgs/image_encodings.h>
#include<iostream>
#include<cstring>
#include<string>
#include<cmath>
#include<stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include "wifi_buff.h"

using namespace std;

bool pass_wifi=false;
int left_temp=0;
int right_temp=0;
serial_common::wifi_buff wifi_send;

void wifi_callback(const serial_common::wifi_buff msg)
{
    wifi_send.left=msg.left;
    wifi_send.right=msg.right;
    cout<<"receive:  left:"<<wifi_send.left<<"  right:"<<wifi_send.right<<endl;
    if(abs(left_temp-wifi_send.left)>=10||abs(right_temp-wifi_send.right)>=10){
      left_temp=wifi_send.left;
      right_temp=wifi_send.right;
      pass_wifi=true;
      cout<<"弹量变化超过10颗 ";
    }
}

int main(int argc, char** argv)
{
   wifi_send.left=0;
   wifi_send.right=0;
   ros::init(argc, argv, "wifi_xjtu");
   ros::NodeHandle nh_;
   ros::Publisher wifi_pub = nh_.advertise<serial_common::wifi_buff>("wifi", 1);

   ros::Subscriber read_wifi = nh_.subscribe("wifi_read", 33,wifi_callback);
  
   while(ros::ok()){
     if(pass_wifi){
     wifi_pub.publish(wifi_send);
     cout<<"弹药量已发送  ";
     cout<<"left:"<<wifi_send.left<<"  right:"<<wifi_send.right<<endl;
     pass_wifi=false;
    }
     ros::spinOnce();
    }
return 0;
}
