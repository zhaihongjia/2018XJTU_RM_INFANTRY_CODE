#include<iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include<math.h>
#include <omp.h>
#include <cctype>
#include <iterator>
#include <ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include </home/xjturm03/catkin_ws/devel/include/serial_common/Infantry.h>
#include "/home/xjturm03/catkin_ws/devel/include/serial_common/Infantry_Read_msg.h"
#include <std_msgs/Empty.h>
using std::cout;
using std::endl;
using namespace cv;
using namespace std;
Mat cam_matrix=(Mat_<float>(3,3)<<987.828,0.0,279.0499,0.0,984.5557,255.5953,0.0,0.0,1.0);
Mat distortion_coeff=(Mat_<float>(4,1)<<-0.641696,6.66807175,-0.0013668,-0.0006792266);
Point2f zuoshang,zuoxia,youshang,youxia;
bool panduan(false);
Point2f *zuoshang_t=new Point2f[20];
Point2f *youshang_t=new Point2f[20];
Point2f *youxia_t=new Point2f[20];
Point2f *zuoxia_t=new Point2f[20];
Point2f daji_2d;
Point3f daji_3d;
Rect *h=new Rect[40];
RotatedRect *juxing_xie=new RotatedRect[40];
Point2f daji_2dxin[20];
Point2f dajiyi[20];
Point2f dajier[20];
cv_bridge::CvImagePtr cv_ptr;
serial_common::Infantry result2;
uint8_t flag_mode=0;
double ckbi[20];
double bizhi;
double an1[20];
double an2[20];
typedef union
{
	uint8_t DATE[10];
	struct{
		uint8_t kaishi;
		uint8_t panduan;
		int16_t xlocation;
		int16_t ylocation;
		int16_t shijie_z;
		uint8_t fankui1;
		uint8_t fankui2;
	}location;
}weizhi;
double juli(Point aa,Point bb)
{
	double ju;
	ju=sqrt((aa.x-bb.x)*(aa.x-bb.x)+(aa.y-bb.y)*(aa.y-bb.y));
	return ju;
}
double calcLineDegree(const Point2f& firstPt, const Point2f& secondPt)
{
    double curLineAngle = 0.0f;
    if (secondPt.x - firstPt.x != 0)
    {
        curLineAngle = atan(static_cast<double>(firstPt.y - secondPt.y) / static_cast<double>(secondPt.x - firstPt.x));
        if (curLineAngle < 0)
        {
            curLineAngle += CV_PI;
        }
    }
    else
    {
        curLineAngle = CV_PI / 2.0f;
    }
    return curLineAngle*180.0f/CV_PI;
};

const uint8_t CRC8_TAB[256] = {
  0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
  0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
  0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
  0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
  0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
  0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
  0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
  0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
  0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
  0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
  0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
  0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
  0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
  0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
  0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
  0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

uint8_t get_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength)
{
	uint8_t ucIndex;
	uint8_t ucCRC8=0xff;
	while (dwLength--)
	  {
		ucIndex = ucCRC8 ^ (*pchMessage++);
		ucCRC8  = CRC8_TAB[ucIndex];
	  }
	  return (ucCRC8);
};

double getdegree(const RotatedRect box)
{
    double degree = 0.0f;
    Point2f vertVect[4];
    box.points(vertVect);
    const double firstLineLen = (vertVect[1].x - vertVect[0].x)*(vertVect[1].x - vertVect[0].x) +
        (vertVect[1].y - vertVect[0].y)*(vertVect[1].y - vertVect[0].y);
    const double secondLineLen = (vertVect[2].x - vertVect[1].x)*(vertVect[2].x - vertVect[1].x) +
        (vertVect[2].y - vertVect[1].y)*(vertVect[2].y - vertVect[1].y);
    if (firstLineLen > secondLineLen)
    {
        degree = calcLineDegree(vertVect[0], vertVect[1]);
    }
    else
    {
        degree = calcLineDegree(vertVect[2], vertVect[1]);
    }
    return degree;
};
void get_distance()
{
	Mat trans_da=(Mat_<float>(3,1));
	Mat trans_xiao=(Mat_<float>(3,1));
	if(bizhi>2.8)
	{
		 Mat trans;
		 Mat r;
		 Mat_<float> zuo;
		vector<cv::Point3f> point3d_da,point3d_xiao;
        vector<cv::Point2f> point2d_da,point2d_xiao;
		point3d_da.push_back(Point3f(-11.5,2,0));
		point3d_da.push_back(Point3f(11.5,2,0));
		point3d_da.push_back(Point3f(11.5,-2,0));
		point3d_da.push_back(Point3f(-11.5,-2,0));
		point2d_da.push_back(zuoshang);
		point2d_da.push_back(youshang);
		point2d_da.push_back(youxia);
		point2d_da.push_back(zuoxia);
		solvePnP(point3d_da,point2d_da,cam_matrix,distortion_coeff,r,trans);
		trans.convertTo(zuo,CV_32F);
		daji_3d.x=zuo.at<float>(0,0);
		daji_3d.y=zuo.at<float>(0,1);
		daji_3d.z=zuo.at<float>(0,2);
	}
	if(bizhi<=2.8)
	{
		 Mat trans=(Mat_<float>(3,1));
		 Mat r;
		 Mat_<float> zuo;
		vector<cv::Point3f> point3d_da,point3d_xiao;
        vector<cv::Point2f> point2d_da,point2d_xiao;
		point3d_da.push_back(Point3f(-6.5,2,0));
		point3d_da.push_back(Point3f(6.5,2,0));
		point3d_da.push_back(Point3f(6.5,-2,0));
		point3d_da.push_back(Point3f(-6.5,-2,0));
		point2d_da.push_back(zuoshang);
		point2d_da.push_back(youshang);
		point2d_da.push_back(youxia);
		point2d_da.push_back(zuoxia);
		solvePnP(point3d_da,point2d_da,cam_matrix,distortion_coeff,r,trans);
		trans.convertTo(zuo,CV_32F);
		daji_3d.x=zuo.at<float>(0,0);
		daji_3d.y=zuo.at<float>(0,1);
		daji_3d.z=zuo.at<float>(0,2);
	}
}
Mat get_erzhi(Mat frame)
{
	Mat element1=getStructuringElement(MORPH_RECT,Size(2,7));
	Mat element2=getStructuringElement(MORPH_RECT,Size(2,3));
	Mat element3=getStructuringElement(MORPH_RECT,Size(1,5));
	imshow("first",frame);
	Mat imahsv;
	imahsv=frame.clone();
	dilate(imahsv,imahsv,element1);
	Mat_<Vec3b>::iterator it=imahsv.begin<Vec3b>();
	Mat_<Vec3b>::iterator itend=imahsv.end<Vec3b>();
	for(;it!=itend;++it){
			int gg=(*it)[0];
			int ggg=(*it)[1];
			int gggg=(*it)[2];
	if((gg-ggg>70&&gg-gggg>70)||(ggg-gg>3&&ggg-gggg>3)||(gggg-ggg>70&&gggg-gg>70)){
				(*it)[0]=0;
				(*it)[1]=0;
				(*it)[2]=0;}}
	erode(imahsv,imahsv,element2);
	erode(imahsv,imahsv,element3);
	imshow("guolv",imahsv);
	Mat huidu;
	cvtColor(imahsv,huidu,CV_BGR2GRAY);
	Mat erzhi;
	threshold(huidu,erzhi,120,255,THRESH_BINARY);
	return erzhi;
}
int main_last()
{
	if(flag_mode>0)
	{
		cout<<"收到数据！"<<flag_mode<<endl;
	}
	weizhi send_data;
	Mat frame = cv_ptr->image;
	{
		Mat erzhi=get_erzhi(frame);
		vector<vector<Point> >contours;
		findContours(erzhi,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
		int shu=contours.size();
		int f=0;
		for(int u=0;u<shu;u++)
		{
			Rect hh=boundingRect(Mat(contours[u]));
			if((hh.br().y-hh.tl().y)/(hh.br().x-hh.tl().x)>=2&&(hh.br().y-hh.tl().y)/(hh.br().x-hh.tl().x)<=6&&hh.br().y-hh.tl().y>6)
			{
			h[f]=hh;
			juxing_xie[f]=minAreaRect(Mat(contours[u]));
		    f++;
			}
		}
	Point temp;
	if(!panduan)
	{
	Point2f ad1,ad2,as1,as2;
	Point2f zuoshang_zheng,youxia_zheng,zuoxia_zheng,youshang_zheng;
	double a1,b1,c1,d1;
	int m=0;
	for(int i=0;i<f;i++)
	{
		int j=i+1;
		for(;j<f;j++)
		{
			ad1=h[i].tl();ad2=h[j].tl();
			as1=h[i].br();as2=h[j].br();
			if(ad1.x>ad2.x){
				zuoshang_zheng=ad2;zuoxia_zheng=as2;
				youxia_zheng=as1;youshang_zheng=ad1;}
			else{
				zuoshang_zheng=ad1,zuoxia_zheng=as1;
				youxia_zheng=as2,youshang_zheng=ad1;}
			a1=(ad1.x+as1.x)/2;b1=(ad1.y+as1.y)/2;
			c1=(ad2.x+as2.x)/2;d1=(ad2.y+as2.y)/2;
			double max1,min1,max2,min2;
			if(h[i].height>h[j].height){
				max1=h[i].height;min1=h[j].height;
			}
			else{
				min1=h[i].height;max1=h[j].height;
			}
			if(h[i].width>h[j].width){
				max2=h[i].width;min2=h[j].width;
			}
			else{
				min2=h[i].width;max2=h[j].width;
			}
		double angle1=getdegree(juxing_xie[i]);
		double angle2=getdegree(juxing_xie[j]);
		if(angle1>90)
			angle1=270-angle1;
		if(angle1<=90)
			angle1=90-angle1;
		if(angle2>90)
			angle2=270-angle2;
		if(angle2<=90)
			angle2=90-angle2;
	if((((((h[i].x-h[j].x)>-6*max1&&(h[i].x-h[j].x)<-3*max1)||((h[i].x-h[j].x)<6*max1&&(h[i].x-h[j].x)>3*max1))&&max1>6)
		||(((h[i].x-h[j].x)>=-3*max1&&(h[i].x-h[j].x)<-1.5*max1)||((h[i].x-h[j].x)<=3*max1&&(h[i].x-h[j].x)>1.5*max1)))
		&&(min1>max1*0.7)&&(min2>max2*0.5)&&(fabs(b1-d1)<0.5*max1)&&(fabs(angle1-angle2)<10||fabs(angle1-angle2)>170))
	{
		an1[m]=fabs(getdegree(juxing_xie[i])-getdegree(juxing_xie[j]));
		    Point2f dingdian1[4],dingdian2[4];
				Point2f temp1,temp2;
				if(h[i].x<h[j].x){
		        juxing_xie[i].points(dingdian1);
				juxing_xie[j].points(dingdian2);
				}
				else{
				juxing_xie[i].points(dingdian2);
				juxing_xie[j].points(dingdian1);
				}
			for(int i=0;i<4;i++)
			{
		        for(int j=0;j<3;j++)
		        {
			  if(dingdian1[j].y>dingdian1[j+1].y){
				temp1=dingdian1[j];
				dingdian1[j]=dingdian1[j+1];
				dingdian1[j+1]=temp1;
			}
				if(dingdian2[j].y>dingdian2[j+1].y){
				temp2=dingdian2[j];
				dingdian2[j]=dingdian2[j+1];
				dingdian2[j+1]=temp2;
			}
		        }
	       }
			zuoshang=0.5*(dingdian1[0]+dingdian1[1]);
			zuoxia=0.5*(dingdian1[2]+dingdian1[3]);
			youshang=0.5*(dingdian2[0]+dingdian2[1]);
			youxia=0.5*(dingdian2[2]+dingdian2[3]);
			dajiyi[m].x=(zuoshang_zheng.x+youxia_zheng.x)*0.5;
		  dajiyi[m].y=(zuoshang_zheng.y+youxia_zheng.y)*0.5;
			bizhi=(youxia_zheng.x-zuoshang_zheng.x)/(zuoxia_zheng.y-zuoshang_zheng.y);
			panduan=true;
			m++;
	}
    }
	}
	int minjuli=0;
	for(int i=0;i<m-1;i++)
	{
		if(an1[minjuli]>an1[i+1])
			minjuli=i+1;
	}
	daji_2d=dajiyi[minjuli];
	}
	if(panduan){
	Point2f ad1,ad2,as1,as2;
	Point2f zuoshang_zheng,youxia_zheng,zuoxia_zheng,youshang_zheng;
	double a1,b1,c1,d1;
	int m=0;
	for(int i=0;i<f;i++)
	{
		int j=i+1;
		for(;j<f;j++)
		{
			ad1=h[i].tl();
			ad2=h[j].tl();
			as1=h[i].br();
			as2=h[j].br();
			if(ad1.x>ad2.x){
				zuoshang_zheng=ad2;zuoxia_zheng=as2;
				youxia_zheng=as1;youshang_zheng=ad1;}
			else{
				zuoshang_zheng=ad1,zuoxia_zheng=as1;
				youxia_zheng=as2,youshang_zheng=ad1;}
			a1=(ad1.x+as1.x)/2;b1=(ad1.y+as1.y)/2;
			c1=(ad2.x+as2.x)/2;d1=(ad2.y+as2.y)/2;
			double max1,min1,max2,min2;
			if(h[i].height>h[j].height){
				max1=h[i].height;min1=h[j].height;
			}
			else{
				min1=h[i].height;max1=h[j].height;
			}
			if(h[i].width>h[j].width){
				max2=h[i].width;min2=h[j].width;
			}
			else{
				min2=h[i].width;max2=h[j].width;
			}
		double angle1=getdegree(juxing_xie[i]);
		double angle2=getdegree(juxing_xie[j]);
		if(angle1>90)
			angle1=270-angle1;
		if(angle1<=90)
			angle1=90-angle1;
		if(angle2>90)
			angle2=270-angle2;
		if(angle2<=90)
			angle2=90-angle2;
	if((((((h[i].x-h[j].x)>-6*max1&&(h[i].x-h[j].x)<-3*max1)||((h[i].x-h[j].x)<6*max1&&(h[i].x-h[j].x)>3*max1))&&max1>6)
		||(((h[i].x-h[j].x)>=-3*max1&&(h[i].x-h[j].x)<-1.2*max1)||((h[i].x-h[j].x)<=3*max1&&(h[i].x-h[j].x)>1.2*max1)))
		&&(min1>max1*0.2)&&(min2>max2*0.2)&&(fabs(b1-d1)<0.6*max1)&&(fabs(angle1-angle2)<20||fabs(angle1-angle2)>160))
	{
		    Point2f dingdian1[4],dingdian2[4];
				Point2f temp1,temp2;
				if(h[i].x<h[j].x){
		        juxing_xie[i].points(dingdian1);
				juxing_xie[j].points(dingdian2);
				}
				else{
				juxing_xie[i].points(dingdian2);
				juxing_xie[j].points(dingdian1);
				}
			for(int i=0;i<4;i++)
			{
		        for(int j=0;j<3;j++)
		        {
			  if(dingdian1[j].y>dingdian1[j+1].y){
				temp1=dingdian1[j];
				dingdian1[j]=dingdian1[j+1];
				dingdian1[j+1]=temp1;
			}
				if(dingdian2[j].y>dingdian2[j+1].y){
				temp2=dingdian2[j];
				dingdian2[j]=dingdian2[j+1];
				dingdian2[j+1]=temp2;
			}
		        }
	       }
			zuoshang_t[m]=0.5*(dingdian1[0]+dingdian1[1]);
			zuoxia_t[m]=0.5*(dingdian1[2]+dingdian1[3]);
			youshang_t[m]=0.5*(dingdian2[0]+dingdian2[1]);
			youxia_t[m]=0.5*(dingdian2[2]+dingdian2[3]);
		  daji_2dxin[m].x=(zuoshang_zheng.x+youxia_zheng.x)*0.5;
		  daji_2dxin[m].y=(zuoshang_zheng.y+youxia_zheng.y)*0.5;
			ckbi[m]=(youxia_zheng.x-zuoshang_zheng.x)/(zuoxia_zheng.y-zuoshang_zheng.y);
		m++;
		}
		}
	}
	double *zxjuli=new double[m];
	for(int i=0;i<m;i++)
	{
		zxjuli[i]=juli(daji_2d,daji_2dxin[i]);
	}
	bizhi=ckbi[0];
	for(int i=0;i<m-1;i++)
	{
		if(ckbi[i]<ckbi[i+1])
		bizhi=ckbi[i+1];
	}
	int minjuli=0;
	for(int i=0;i<m-1;i++)
	{
		if(zxjuli[minjuli]>zxjuli[i+1])
			minjuli=i+1;
	}
	temp=daji_2d;
	daji_2d=daji_2dxin[minjuli];
	if(m==0)
	daji_2d=Point2f(0.0,0.0);
	zuoshang=zuoshang_t[minjuli];
	zuoxia=zuoxia_t[minjuli];
	youshang=youshang_t[minjuli];
	youxia=youxia_t[minjuli];
	if((juli(temp,daji_2d)>30||daji_2d==Point2f(0.0,0.0))){
	Point2f ad1,ad2,as1,as2;
	Point2f zuoshang_zheng,youxia_zheng,zuoxia_zheng,youshang_zheng;
	double a1,b1,c1,d1;
	int m=0;
	for(int i=0;i<f;i++)
	{
		int j=i+1;
		for(;j<f;j++)
		{
			ad1=h[i].tl();
			ad2=h[j].tl();
			as1=h[i].br();
			as2=h[j].br();
			if(ad1.x>ad2.x){
				zuoshang_zheng=ad2;zuoxia_zheng=as2;
				youxia_zheng=as1;youshang_zheng=ad1;}
			else{
				zuoshang_zheng=ad1,zuoxia_zheng=as1;
				youxia_zheng=as2,youshang_zheng=ad1;}
			a1=(ad1.x+as1.x)/2;b1=(ad1.y+as1.y)/2;
			c1=(ad2.x+as2.x)/2;d1=(ad2.y+as2.y)/2;
			double max1,min1,max2,min2;
			if(h[i].height>h[j].height){
				max1=h[i].height;min1=h[j].height;
			}
			else{
				min1=h[i].height;max1=h[j].height;
			}
			if(h[i].width>h[j].width){
				max2=h[i].width;min2=h[j].width;
			}
			else{
				min2=h[i].width;max2=h[j].width;
			}
		double angle1=getdegree(juxing_xie[i]);
		double angle2=getdegree(juxing_xie[j]);
		if(angle1>90)
			angle1=270-angle1;
		if(angle1<=90)
			angle1=90-angle1;
		if(angle2>90)
			angle2=270-angle2;
		if(angle2<=90)
			angle2=90-angle2;
       if((((((h[i].x-h[j].x)>-6*max1&&(h[i].x-h[j].x)<-3*max1)||((h[i].x-h[j].x)<6*max1&&(h[i].x-h[j].x)>3*max1))&&max1>6)
		||(((h[i].x-h[j].x)>=-3*max1&&(h[i].x-h[j].x)<-1.5*max1)||((h[i].x-h[j].x)<=3*max1&&(h[i].x-h[j].x)>1.5*max1)))
		&&(min1>max1*0.7)&&(min2>max2*0.5)&&(fabs(b1-d1)<0.5*max1)&&(fabs(angle1-angle2)<10||fabs(angle1-angle2)>170))
	{
		an2[m]=fabs(getdegree(juxing_xie[i])-getdegree(juxing_xie[j]));
		    Point2f dingdian1[4],dingdian2[4];
				Point2f temp1,temp2;
				if(h[i].x<h[j].x){
		        juxing_xie[i].points(dingdian1);
				juxing_xie[j].points(dingdian2);
				}
				else{
				juxing_xie[i].points(dingdian2);
				juxing_xie[j].points(dingdian1);
				}
			for(int i=0;i<4;i++)
			{
		        for(int j=0;j<3;j++)
		        {
			  if(dingdian1[j].y>dingdian1[j+1].y){
				temp1=dingdian1[j];
				dingdian1[j]=dingdian1[j+1];
				dingdian1[j+1]=temp1;
			}
				if(dingdian2[j].y>dingdian2[j+1].y){
				temp2=dingdian2[j];
				dingdian2[j]=dingdian2[j+1];
				dingdian2[j+1]=temp2;
			}
		        }
	       }
			zuoshang=0.5*(dingdian1[0]+dingdian1[1]);
			zuoxia=0.5*(dingdian1[2]+dingdian1[3]);
			youshang=0.5*(dingdian2[0]+dingdian2[1]);
			youxia=0.5*(dingdian2[2]+dingdian2[3]);
			dajier[m].x=(zuoshang_zheng.x+youxia_zheng.x)*0.5;
		  dajier[m].y=(zuoshang_zheng.y+youxia_zheng.y)*0.5;
			bizhi=(youxia_zheng.x-zuoshang_zheng.x)/(zuoxia_zheng.y-zuoshang_zheng.y);
			m++;
	}
    }
	}
	int minjuli=0;
	for(int i=0;i<m-1;i++)
	{
		if(an2[minjuli]>an2[i+1])
			minjuli=i+1;
	}
	if(m!=0)
	daji_2d=dajier[minjuli];
	if(m==0)
	daji_2d=Point2f(0.0,0.0);
	}
	}
  get_distance();
	circle(frame,daji_2d,2.5,Scalar(0,0,255),2.5);
	imshow("result",frame);
	if(daji_2d.x!=0||daji_2d.y!=0){
	send_data.location.kaishi=0x01;
	send_data.location.panduan=0x01;
	send_data.location.xlocation=daji_2d.x;
	send_data.location.ylocation=480-daji_2d.y;
	send_data.location.shijie_z=daji_3d.z;
	send_data.location.fankui1=0x65;
	send_data.location.fankui2=0x66;
	}
	else{
	send_data.location.kaishi=0x01;
	send_data.location.panduan=0x00;
	send_data.location.xlocation=0;
	send_data.location.ylocation=0;
	send_data.location.shijie_z=0;
	send_data.location.fankui1=0x65;
	send_data.location.fankui2=0x66;
	}
	result2.kaishi=send_data.location.kaishi;
	result2.panduan=send_data.location.panduan;
	result2.xlocation=send_data.location.xlocation;
	result2.ylocation=send_data.location.ylocation;
	result2.shijie_z=send_data.location.shijie_z;
	result2.fankui1=send_data.location.fankui1;
	result2.fankui2=send_data.location.fankui2;
	result2.crcnumber=get_crc8_check_sum(send_data.DATE,10);
ROS_INFO_STREAM("ORIGINALDATA:"<<(int16_t)result2.xlocation);
ROS_INFO_STREAM("ORIGINALDATA:"<<(int16_t)result2.ylocation);
ROS_INFO_STREAM("ORIGINALDATA:"<<(int16_t)result2.shijie_z);
cout<<endl;
	 }
}

//---------------------------------------------------------------------
void read_callback(const serial_common::Infantry_Read_msg::ConstPtr& msg)
{
  flag_mode = msg-> mode;
  ROS_INFO_STREAM("RECEIVE: "<< flag_mode);
}
//---------------------------------------------------------------------

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher write_pub = nh_.advertise<serial_common::Infantry>("write", 33);
	ros::Subscriber read_pub = nh_.subscribe("read", 33,read_callback);

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	  main_last();
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
	  write_pub.publish(result2);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
