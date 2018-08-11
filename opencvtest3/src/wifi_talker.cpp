#include <ros/ros.h>
#include <wifi_buff.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#define SERVPORT 10080
#define BACKLOG 10
#define MAXSIZE 1024
using namespace std;

int main(int argc, char **argv)
{

        int sockfd, client_fd;
	struct sockaddr_in my_addr;
	struct sockaddr_in remote_addr;
	//创建套接字
        if ((sockfd = socket( AF_INET, SOCK_STREAM, 0 ) )== -1 ){
		perror("socket create failed!");
		exit(1);
	}
	//绑定端口地址
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(SERVPORT);
	my_addr.sin_addr.s_addr = INADDR_ANY;
	bzero(&(my_addr.sin_zero), 8);
	if (bind(sockfd, (struct sockaddr*) &my_addr, sizeof(struct sockaddr))== -1) {
		perror("bind error!");
		exit(1);
	}
	//监听端口
	if (listen(sockfd, BACKLOG) == -1) {
		perror("listen error");
		exit(1);
	}


  //用于解析ROS参数，第三个参数为本节点名
  ros::init(argc, argv, "wifi_talker");
  ros::NodeHandle nh;
  serial_common::wifi_buff msg;
  msg.left=0;
  msg.right=0;
  ros::Publisher pub = nh.advertise<serial_common::wifi_buff>("wifi_read", 1);
  ros::Rate loop_rate(1.0);

  while (1)
  {

    //int sin_size = sizeof(struct sockaddr_in);
     socklen_t sin_size = sizeof(struct sockaddr_in);
if ((client_fd = accept(sockfd,(struct sockaddr*) &remote_addr,&sin_size)) == -1){
			perror("accept error!");
			continue;
		}




                        char buf[MAXSIZE];
			//接受client发送的请示信息
			int rval;

			if ((rval = read(client_fd, buf, MAXSIZE)) < 0) {
				perror("reading stream error!");
				continue;
			}


			string s(&buf[0],&buf[strlen(buf)]);

                        int number=0;
                        istringstream stream_ck;
                        stream_ck.str(s);
                        stream_ck>>number;

                        msg.right=number%1000;
                        msg.left=number/1000;

                        cout<<"发布的数据为"<<endl;
                        cout<<"left:"<<msg.left<<"  ";
                        cout<<"right:"<<msg.right<<endl;

			close(client_fd);


                        pub.publish(msg);

    loop_rate.sleep();
}
  return 0;
}
