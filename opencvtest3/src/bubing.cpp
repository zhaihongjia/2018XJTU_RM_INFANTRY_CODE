#include <ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<iostream>
#include<cstring>
#include<string>
#include<cmath>
#include<stdlib.h>
#include "classfication.h"
#include "AngleSolver.hpp"
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;
//------------------自动打击---头文件------------------
#include <fstream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <omp.h>
#include <cctype>
#include <iterator>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
//-----------------------------------------------


//---------------电控通讯添加--------------------------
#include "EnemyPos.h"
#include "Infantrymode.h"
//---------------------------------------------------

//手写数字模型
string model_file3 = "/home/xjturm/Desktop/my_code_data/lenet.prototxt";
string trained_file3 = "/home/xjturm/Desktop/my_code_data/lenet_iter_10000.caffemodel";
string trained_file1 = "/home/xjturm/Desktop/my_code_data/mynet_iter_10000.caffemodel";
string mean_file3 = "/home/xjturm/Desktop/my_code_data/mean.binaryproto";
string label_file3 = "/home/xjturm/Desktop/my_code_data/label.txt";

Classifier classfy1(model_file3, trained_file1, mean_file3, label_file3);

//火焰数字模型
string model2_prototxt="/home/xjturm/Desktop/models/big/lenet3.prototxt";
string model2_model="/home/xjturm/Desktop/models/big/_iter_10000.caffemodel";
string model2_mean="/home/xjturm/Desktop/models/big/mean.binaryproto";
string model2_label="/home/xjturm/Desktop/models/big/label.txt";

Classifier classfy2(model2_prototxt,model2_model,model2_mean,model2_label);

//------------------摄像头参数------------------------------
//---new ---cap one-------------------------------------
   Mat intrinsic_Matrix=(Mat_<double>(3,3)<<501.9406,0.0,330.2514,0.0,502.2422,234.5579,0.0,0.0,1.0);
   Mat distortion_coefficients=(Mat_<double>(5,1)<<0.0266,-0.0613,0.0,0.0,-0.0015);

//----new  cap two-------------------------------------
 // Mat intrinsic_Matrix=(Mat_<double>(3,3)<<503.3678,0.0,334.3404,0.0,502.3209,218.9074,0.0,0.0,1.0);
  //Mat distortion_coefficients=(Mat_<double>(5,1)<<0.0571,-0.2157,0.0,0.0,0.2715);
//----------------------------------------------------

// //---- new cap three-------------------------------------
   // Mat intrinsic_Matrix=(Mat_<double>(3,3)<<506.7971,0.0,340.1253,0.0,506.0581,230.8526,0.0,0.0,1.0);
   // Mat distortion_coefficients=(Mat_<double>(5,1)<<0.0498,-0.1089,0.0,0.0,-0.1271);
//----------------------------------------------------

//---new ---cap four-------------------------------------
  //Mat intrinsic_Matrix=(Mat_<double>(3,3)<<505.7416,0.0,336.4507,0.0,506.4035,221.4206,0.0,0.0,1.0);
  //Mat distortion_coefficients=(Mat_<double>(5,1)<<0.0710,-0.2905,0.0,0.0,0.5693);


  // //-----录制视频设置------
  // VideoWriter cam;
  // char filename[64];
  // char extendname[] = ".avi";
  // int record_num=1000;
  // //---------------------

  int flag_mode=10;//默认以10初始话
  int now_mode=10; //电控现在发送的模式
  int pre_mode=10; //电控前一次发送的模式
  //mode=1 火焰数字
  //mode=2 手写数字
  //mode=3 自动打击
  //mode=4 退出大小符，自动打击模式
  //mode=0 开机传输数据




  //中心大宫格大小
  #define b_width 740
  #define b_height 440
  #define dep_num 1
  //----------大小符----------需要更改的参数--------------------------
  #define code_thresold_value 175  //------数码管二值化参数----

  //-----火焰数字参数-----
  #define big_threshold_value 60  //------火焰数字二值化参数----

  //------手写数字参数--
  #define small_rect9_threshold 60  //--九宫格二值化参数--
  #define small_code_value 70      //--手写数字二值参数--

  //---火焰数字---从数码管区域找九宫格区域参数设定---
  #define big_width_ratio 8.1
  #define big_height_ratio 5.1
  #define big_left_ratio 2.3
  #define big_down_ratio 1.0

  //---火焰数字找数码管转hsv图像的比例---第一个矩形框
  #define num_width_ratio 5.5
  #define num_height_ratio 1.5
  #define cut_ratio 0.25

  //----------手写数字查找数码管-----------
  #define small_up_ratio 1.2
  #define small_right_ratio 0.9
  #define small_height_ratio 1.0
  #define small_width_ratio 0.4      //-----越小越宽


  //----------------------------------------------------------------------------------------


int now_code[5] = { 0 };
int pre_code[5] = { 0 };
int goal_index = 0;
int record_index = 0;
int index_rec[dep_num] = { 0 };

//----脚本位置-------
char arg_buff[50]="/home/xjturm/Buff.sh";
char arg_armour[50]="/home/xjturm/Armour.sh";

//是否可以传输数据
bool if_send=false;
//判断第几次进入打符模式
bool if_first=true;


//------------ros 相关-------------------------------------
cv_bridge::CvImagePtr cv_ptr;
//--------------串口通讯----------------------------
serial_common::EnemyPos buff_msg;
serial_common::EnemyPos enemy_msg;
//---------------------------------------------------------


//------------大小符 函数------------------------------------
void sort_rect(vector<Rect> &rect)
{
	int i, j;
  //--------九宫格按照y坐标排序------------
	for (i = 0; i < 8; i++)
	{
		int k = i;
		for (j = i + 1; j < 9; j++)
		{
			if (rect[j].tl().y < rect[k].tl().y)
				k = j;
		}
		if (k != i)
		{
			Rect z;
			z = rect[k];
			rect[k] = rect[i];
			rect[i] = z;
		}
	}

	for (i = 0; i < 2; i++)
	{
		int k = i;
		for (j = i + 1; j < 3; j++)
		{
			if (rect[j].tl().x < rect[k].tl().x)
				k = j;
		}
		if (k != i)
		{
			Rect z;
			z = rect[k];
			rect[k] = rect[i];
			rect[i] = z;
		}
	}

	for (i = 3; i < 5; i++)
	{
		int k = i;
		for (j = i + 1; j < 6; j++)
		{
			if (rect[j].tl().x < rect[k].tl().x)
				k = j;
		}
		if (k != i)
		{
			Rect z;
			z = rect[k];
			rect[k] = rect[i];
			rect[i] = z;
		}
	}

	for (i = 6; i < 8; i++)
	{
		int k = i;
		for (j = i + 1; j < 9; j++)
		{
			if (rect[j].tl().x < rect[k].tl().x)
				k = j;
		}
		if (k != i)
		{
			Rect z;
			z = rect[k];
			rect[k] = rect[i];
			rect[i] = z;
		}
	}
}

int hit_big_index(Mat image,vector<Rect> rect9,int goal)
{
    int index=0;
    vector<Prediction> prediction[9];
    for (int i = 0; i < rect9.size(); ++i)
    {
        Mat result;
        resize(image(rect9[i]),result,Size(28,28));
        threshold(result,result,80,255,CV_THRESH_BINARY);
        prediction[i]=classfy2.Classify(result,1);

        if (atoi(prediction[i][0].first.c_str()) == goal)
        {
            index = i;
        }
    }
    return index;
}

int hit_small_index(Mat image,vector<Rect> rect9,int goal)
{
    int index=0;
    vector<Prediction> prediction[9];
    Mat binary;
    threshold(image,binary,small_code_value,255,CV_THRESH_BINARY_INV);
    for (int i = 0; i < rect9.size(); ++i)
    {
        Mat result=binary(Rect(rect9[i].tl().x+3,rect9[i].tl().y+3,rect9[i].width-6,rect9[i].height-6));

        Mat exp_roi(Size(result.cols + 10, result.rows + 20), CV_8UC1, Scalar(0));
        result.copyTo(exp_roi(Rect(4, 9, result.cols, result.rows)));
        resize(exp_roi,exp_roi,Size(28,28));
        prediction[i]=classfy1.Classify(exp_roi,1);

        if (atoi(prediction[i][0].first.c_str()) == goal)
        {
            index = i;
        }
    }
    return index;
}

int choose_best(int *q)
{
    int i, j, c, mc = 0, index = dep_num - 1;;
    for (i = 0; i<dep_num; i++)
    {
        c = 0;
        for (j = 0; j<dep_num; j++)
        {
            if (*(q + i) == *(q + j))
            {
                c++;
            }
        }
        if (c>mc)
        {
            mc = c;
            index = i;
        }
    }
    if(*(q + index)!=*q)
    {
      cout<<"----------------------------------和第一个不一样！"<<endl;
    }
    return *(q + index);
}

bool codechange(int* now_code, int* pre_code)
{
    int i = 0, change_num = 0;
    for (i = 0; i < 5; i++)
    {
        if (now_code[i] != pre_code[i])
            change_num++;
    }
    if (change_num > 2)
        return true;
    else
        return false;
}

void read_code(Mat img, vector<Rect> &rect, int *number)
{
    number[5] = { 0 };
    int i;
    for (i = 0; i < 5; i++) {
        if ((double)rect[i].height / (double)rect[i].width >= 2.3)
            number[i] = 1;
        else {
            int all = 0, m;
            int x1 = rect[i].tl().x, x2 = x1 + rect[i].width / 2, x3 = x1 + rect[i].width;
            int y1 = rect[i].tl().y, y2 = y1 + rect[i].height / 3, y3 = y1 + rect[i].height * 2 / 3, y4 =
                    y1 + rect[i].height;
            int p[7][4] = { { x2 - 0.1 * rect[i].width, y1, x2 - 0.1 * rect[i].width, y2 },
                            { x2 - 0.1 * rect[i].width, y2, x3, y2 },
                            { x2 - 0.1 * rect[i].width, y3, x3, y3 },
                            { x2 - 0.1 * rect[i].width, y3, x2 - 0.1 * rect[i].width, y4 },
                            { x1, y3, x2 - 0.1 * rect[i].width, y3 },
                            { x1, y2, x2 - 0.1 * rect[i].width, y2 },
                            { x2, y2, x2, y3 } };
            for (m = 0; m < 7; m++) {
                bool iswhite = false;
                int j;
                if (p[m][1] == p[m][3]) {
                    uchar *data = img.ptr<uchar>(p[m][1]);
                    for (j = 0; j <= img.cols; j++) {
                        if (data[j] > 100 && j <= p[m][2] && j >= p[m][0]) {
                            iswhite = true;
                            break;
                        }
                        //data[j]=255;
                    }
                }
                else if (p[m][0] == p[m][2]) {

                    for (j = p[m][1]; j <= p[m][3]; j++) {
                        uchar *data = img.ptr<uchar>(j);
                        if (data[p[m][0]] > 100 && j <= p[m][3] && j >= p[m][1]) {
                            iswhite = true;
                            break;
                        }
                        //data[p[m][0]]=255;
                    }
                }
                else {
                    cout << "输入坐标有误！" << endl;
                }
                if (iswhite)
                    all = all + (int)pow(2, m);
            }
            switch (all) {
                case 63:
                    number[i] = 0;
                    break;
                case 91:
                    number[i] = 2;
                    break;
                case 79:
                    number[i] = 3;
                    break;
                case 102:
                    number[i] = 4;
                    break;
                case 109:
                    number[i] = 5;
                    break;
                case 125:
                    number[i] = 6;
                    break;
                case 7:
                    number[i] = 7;
                    break;
                case 127:
                    number[i] = 8;
                    break;
                case 111:
                    number[i] = 9;
                    break;
                default:
                    number[i] = -1;
            }
        }
    }
    imshow("read_code_img:",img);
    cout << "数码管密码:" << number[0] << number[1] << number[2] << number[3] << number[4] << endl;
    //return  number[0] * 10000 + number[1] * 1000 + number[2] * 100 + number[3] * 10 + number[4];
}

void get_small_rect(Mat img)
{
    vector<Rect> rect5;
    vector<Rect> rect9;

    rect9.clear();
    rect5.clear();

    //------------------------------------找九宫格预处理
    vector<vector<Point> > contours;
    Mat gray, binary, dilated;
    cvtColor(img, gray, CV_BGR2GRAY);
    //-----------------------------------找九宫格不用腐蚀---
    threshold(gray, binary, small_rect9_threshold, 255, CV_THRESH_BINARY);
    findContours(binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    // //-----------------------------------对应的显示九宫格图片---
    // Mat s_con(img.size(), CV_8UC1, Scalar(255));


    //-----------------------------------寻找九宫格
    int i;
    for (i = 0; i < contours.size(); i++) {
        Rect poly = boundingRect(contours[i]);
        double area = contourArea(contours[i]);
        double wh_ratio = (double) poly.width / poly.height;
        //cout << "area:" << area << "   ratio:" << wh_ratio << endl;
        //-----一般区域面积在5000-8000
        if ((area < 12000 && area > 2000 && wh_ratio > 1.4 && wh_ratio <= 2.4)&&(poly.tl().y>0.09*img.rows&&poly.tl().x>2&&poly.br().x<(img.cols-2)&&poly.br().y<(img.rows-2))) {
            rect9.push_back(poly);
            //cout<<"poly:"<<poly.tl().x<<"   "<<poly.tl().y<<endl;
            //cout << "poly_height:" << poly.height << "poly_width:" << poly.width << endl;
            //cout << "手写数字九宫格：area:" << area << "   ratio:" << wh_ratio << endl;
        }
    }

    //----------------------------------删除多余九宫格---优先排序并删除多余的---主要是排除旁边的小矩形
    if (rect9.size() > 9) {
        //cout << rect9.size() << endl;
        for (i = 0; i < rect9.size() - 1; i++) {
            int k = i;
            for (int j = i + 1; j < rect9.size(); j++) {
                if (rect9[k].height < rect9[j].height)
                    k = j;
            }
            if (k != i) {
                Rect exc = rect9[i];
                rect9[i] = rect9[k];
                rect9[k] = exc;
            }
        }
    }
    while (rect9.size() > 9) {
        rect9.erase(rect9.begin() + 9);
    }

    if (rect9.size() == 9) {
        // //---------------------------------------画九宫格
        // for (i = 0; i < rect9.size(); i++) {
        //     rectangle(s_con, rect9[i], Scalar(0));
        // }
        // imshow("small_rect9:", s_con);

        //---------------------------------------九宫格排序
        sort_rect(rect9);

        double first_height = 1.2*rect9[0].height;
        //--------------------------------------找数码管区域
        //double number_height=min((double)rect9[0].tl().y,small_up_ratio * first_height);
        if ((rect9[0].tl().y - small_up_ratio * first_height > 0) &&
            (rect9[0].tl().x + small_right_ratio * first_height < img.cols) &&
            (rect9[2].tl().x - rect9[0].tl().x-small_width_ratio * first_height> 0)) {
            //-----------------------------------直接用二值化的结果
            Mat rois = gray(Rect(rect9[0].tl().x + small_right_ratio*first_height, rect9[0].tl().y - small_up_ratio*first_height, rect9[2].tl().x - rect9[0].tl().x - small_width_ratio*first_height, small_height_ratio*first_height));

            //------------------------------------hsv的结果
 //            Mat rois = img(Rect(rect9[0].tl().x + small_right_ratio * first_height,
 //                                rect9[0].tl().y -  small_up_ratio * first_height,
 //                                rect9[2].tl().x - rect9[0].tl().x - small_width_ratio * first_height,
 //                                small_height_ratio * first_height));
 //            rois = hsv_change(rois);

            Mat num_dilate, num_binary;
            // Mat num_check(rois.size(), CV_8UC1, Scalar(255));//----画数码w管矩形的图片----

            //--------------------------膨胀数码管--------------------------注意：官方数码管比我们小，间隔缝较小    --------影响筛选条件   数字的hw_ratio-
            threshold(rois, num_binary, code_thresold_value, 255, CV_THRESH_BINARY);
            // imshow("small_code_binary", num_binary);
            Mat element(Size(1, 5), CV_8UC1, Scalar(1));
            dilate(num_binary, num_dilate, Mat());
            //dilate(num_dilate, num_dilate, Mat());
            dilate(num_dilate, num_dilate, element);
            //dilate(num_dilate, num_dilate, element);
            // imshow("small_code_dilated", num_dilate);


            vector<vector<Point> > num_contours;
            findContours(num_dilate, num_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
            for (i = 0; i < num_contours.size(); i++) {
                Rect poly = boundingRect(num_contours[i]);
                double area = contourArea(num_contours[i]);
                double hw_ratio = (double) poly.height / poly.width;
                //cout << "area:" << area << "   ratio:" << hw_ratio << endl;
                //cout << "height:" << poly.height << endl;
                if (((area < 3000 && area > 200 && hw_ratio > 1.0 && hw_ratio <= 2.5) ||
                     (area < 850 && area > 80 && hw_ratio > 2.5))) {
                    rect5.push_back(poly);
                    //cout<<"poly:"<<poly.tl().x<<"   "<<poly.tl().y<<endl;
                    //cout << "area:" << area << "   ratio:" << hw_ratio << endl;
                }
            }

            if (rect5.size() < 5) {
                cout << "----------------密码个数小于五个！" << endl;
            } else if (rect5.size() < 9) {
                if (rect5.size() > 5) {
                    //-------------计算数码管的优先级--------按中心y坐标差值排序
                    float dises[9] = {0};
                    int size = (int) rect5.size(), j;
                    for (i = 0; i < size; i++) {
                        for (j = 0; j < size; j++) {
                            float d = abs((rect5[i].tl().y + rect5[i].br().y) / 2 -
                                                (rect5[j].tl().y + rect5[j].br().y) / 2);
                            dises[i] += d;
                        }
                    }

                    for (i = 0; i < size - 1; i++) {
                        int k = i;
                        for (j = i + 1; j < size; j++) {
                            if (dises[j] < dises[k])
                                k = j;
                        }
                        if (k != i) {
                            float z=dises[i];
                            dises[i]=dises[k];
                            dises[k]=z;
                            Rect exc = rect5[i];
                            rect5[i] = rect5[k];
                            rect5[k] = exc;
                        }
                    }
                    //----------------------------------------去除多余的数码管
                    while (rect5.size() > 5) {
                        rect5.erase(rect5.begin() + 5);
                    }
                }
                // //--------------------------------------------画数码管矩形
                // for (i = 0; i < 5; i++) {
                //     rectangle(num_check, rect5[i].tl(), rect5[i].br(), Scalar(0), 1, 8);
                // }
                // imshow("small_num_check", num_check);

                //--------------------------------------------数码管排序
                for (i = 0; i < rect5.size() - 1; i++) {
                    int k = i;
                    for (int j = i + 1; j < rect5.size(); j++) {
                        if (rect5[j].tl().x < rect5[k].tl().x)
                            k = j;
                    }
                    if (k != i) {
                        Rect exc = rect5[i];
                        rect5[i] = rect5[k];
                        rect5[k] = exc;
                    }
                }

                //----------------------------------------读取数码管数字
                read_code(num_binary, rect5, now_code);

                //----------------------------------------检测密码是否变化
                if (codechange(now_code, pre_code)) {
                    goal_index = 0;
                }

                //---------------------------------------------密码更新
                for (i = 0; i < 5; i++) {
                    pre_code[i] = now_code[i];
                }

                int goal = now_code[goal_index];
                index_rec[record_index] = hit_small_index(gray, rect9, goal);
                record_index++;
                if (record_index == dep_num) {
                    record_index = record_index % dep_num;
                    goal_index++;
                    if (goal_index == 5)
                        goal_index = goal_index % 5;

            int best_record;
            best_record = choose_best(index_rec);
            // cout<<"best_record"<<best_record<<endl;

            vector<Point2f> points2d;
            vector<Point3f> points3d;

            float x_offset=0;
            float y_offset=0;

            points2d.clear();
            points2d.push_back(Point2f((rect9[0].tl().x+rect9[0].br().x)/2+x_offset,(rect9[0].tl().y+rect9[0].br().y)/2+y_offset));
            points2d.push_back(Point2f((rect9[2].tl().x+rect9[2].br().x)/2+x_offset,(rect9[2].tl().y+rect9[2].br().y)/2+y_offset));
            points2d.push_back(Point2f((rect9[8].tl().x+rect9[8].br().x)/2+x_offset,(rect9[8].tl().y+rect9[8].br().y)/2+y_offset));
            points2d.push_back(Point2f((rect9[6].tl().x+rect9[6].br().x)/2+x_offset,(rect9[6].tl().y+rect9[6].br().y)/2+y_offset));

            points3d.clear();
            points3d.push_back(Point3f(-b_width/2, -b_height/2, 0));
            points3d.push_back(Point3f(b_width/2, -b_height/2, 0));
            points3d.push_back(Point3f(b_width/2, b_height/2, 0));
            points3d.push_back(Point3f(-b_width/2, b_height/2, 0));

            Mat r, trans;
            solvePnP(points3d, points2d, intrinsic_Matrix, distortion_coefficients, r, trans);

            double *my_xyz = (double *)trans.data;
            double distance=my_xyz[2];

            int16_t xangle;
            int16_t yangle;

            distance=distance/10;
            xangle=0.0;
            yangle=0.0;
            if(best_record==6||best_record==7||best_record==8)
            {
                yangle=(-0.4*distance+120);
            }
            else if(best_record==0||best_record==1||best_record==2)
            {
                yangle=(-1)*((-0.4)*distance+120);
            }
            if(best_record==0||best_record==3||best_record==6)
            {
              xangle=(-1)*((-0.97)*distance+260);
            }
            else if(best_record==2||best_record==5||best_record==8)
            {
              xangle=(-0.97)*distance+260;
            }

            buff_msg.enemy_dist = distance;
            buff_msg.enemy_yaw = xangle;
            buff_msg.enemy_pitch = yangle;
            buff_msg.mode=flag_mode;

            //可以传送数据
            if_send=true;
			}
    }
  }
		else {
			cout << "-------------------------数码管的区域有误！" << endl;
		}
	}
	else {
		cout << "---------------------九宫格个数不对！" << endl;
	}
}

int get_big_rect(Mat img)
{
  vector<Rect> rect5;
   vector<Rect> rect9;
   rect5.clear();
   rect9.clear();

   //-----------------------------------------寻找数码管矩形 ---需要形态学操作
   vector<vector<Point> > contours;
   Mat gray, binary, dilated;
   cvtColor(img, gray, CV_BGR2GRAY);
   //------------------设置找数码管的参数： 二值化阈值  与  膨胀的次数一起改变查找效果--影响数码管的面积参数----
   threshold(gray, binary, code_thresold_value, 255, CV_THRESH_BINARY);
   Mat element(Size(1, 3), CV_8UC1, Scalar(1));
   dilate(binary, dilated, Mat());
   //dilate(dilated, dilated, element);
   //dilate(dilated, dilated, element);
   // imshow("big_code_dilated", dilated);
   // imshow("big_code_binary", binary);
   findContours(dilated, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
   //-----------------------------------------查看数码管轮廓矩形--------------------------------
   Mat s_con(img.size(), CV_8UC1, Scalar(255));


   //------------------------------------------按照条件筛选数码管----------------------------
   int i;
   for (i = 0; i < contours.size(); i++) {
       Rect poly = boundingRect(contours[i]);
       double area = contourArea(contours[i]);
       double hw_ratio = (double) poly.height / poly.width;
       //cout << "筛选前：area:" << area << "   ratio:" << hw_ratio << endl;
       //----------筛选数码管条件  面积  高度  在img中的位置------------------条件放宽一点--------------
       //-------数字1--面积在370左右，  其他数字面积在500-1500  比例1.3左右
       if (((area < 3000 && area > 180 && hw_ratio > 1.0 && hw_ratio <= 2.2) ||
            (area < 850 && area > 80 && hw_ratio > 2.2&& hw_ratio <3.5 )) && poly.tl().y < 0.44 * img.rows &&
           poly.tl().x > 0.1 * img.cols && poly.tl().x < 0.9 * img.cols && poly.tl().y > 2) {
           rect5.push_back(poly);
           //rectangle(s_con, poly.tl(), poly.br(), Scalar(0), 1, 8);
           //cout << "poly_height:" << poly.height << " poly_width:" << poly.width << endl;
           //cout << "area:" << area << "   ratio:" << hw_ratio << endl;
           //cout<<"height_ratio:"<<(double)poly.tl().y/img.rows<<"   width_ratio:"<<(double)poly.tl().x/img.cols<<endl;
       }
   }

   if (rect5.size() < 5) {
       cout << "-------------火焰数字数码管密码个数小于五个！" << endl;
       return 0;
   }//----------------------------------------------------当数码管可能个数在5-9之间时
   else if (rect5.size() < 16) {
       int size = (int) rect5.size(), j;

       if (rect5.size() > 5&&rect5.size()<9) {
           float dises[9] = {0};
           for (i = 0; i < size; i++) {
               for (j = 0; j < size; j++) {
                   //-------------------------------------竖直高度差 优先级计算--------
                   float d = abs(rect5[i].tl().y - rect5[j].tl().y);
                   //float d = 800 * abs((rect5[i].tl().y + rect5[i].br().y) / 2 - (rect5[j].tl().y + rect5[j].br().y) / 2) + 0*abs((rect5[i].tl().x + rect5[i].br().x) / 2 - (rect5[j].tl().x + rect5[j].br().x) / 2);
                   dises[i] = dises[i] + d;
               }
           }

           //---------------------------------------------进行优先级排序-----rect9和dises都要排序
           for (i = 0; i < rect5.size() - 1; i++) {
               int k = i;
               for (j = i + 1; j < rect5.size(); j++) {
                   if (dises[j] < dises[k])
                       k = j;
               }

               if (k != i) {
                   float zz = dises[i];
                   dises[i] = dises[k];
                   dises[k] = zz;
                   Rect exc = rect5[i];
                   rect5[i] = rect5[k];
                   rect5[k] = exc;
               }
           }
           while (rect5.size() > 5) {
               rect5.erase(rect5.begin() + 5);
           }
       }

       else
       {
           //cout<<"---------------------------------火焰数字数码管大于9小于16！"<<endl;
           //---------------------------------------------进行优先级排序-----rect9和dises都要排序
           for (i = 0; i < rect5.size() - 1; i++) {
               int k = i;
               for (j = i + 1; j < rect5.size(); j++) {
                   if(rect5[j].tl().y<rect5[k].tl().y)
                       k = j;
               }
               if (k != i) {
                   Rect exc = rect5[i];
                   rect5[i] = rect5[k];
                   rect5[k] = exc;
               }
           }
           while (rect5.size() > 5) {
               rect5.erase(rect5.begin() + 5);
           }
       }

       //-----------------------------------------------数码管从左到右排序
       for (i = 0; i < rect5.size() - 1; i++) {
           int k = i;
           for (j = i + 1; j < rect5.size(); j++) {
               if (rect5[j].tl().x < rect5[k].tl().x)
                   k = j;
           }
           if (k != i) {
               Rect exc = rect5[i];
               rect5[i] = rect5[k];
               rect5[k] = exc;
           }
       }

       //----------------------------------------------画出数码管矩形
       for (i = 0; i < rect5.size(); i++) {
           rectangle(s_con, rect5[i].tl(), rect5[i].br(), Scalar(0), 1, 8);
       }


       //---直接二值识别-----
       read_code(binary, rect5, now_code);

       // imshow("big_s_con", s_con);

       //-------------------------------------------找九宫格-------------------------------------------------------------------------------------

       //-------------固定maxheight比例---------------
       //float max_height = max(rect5[0].height, max(rect5[1].height, max(rect5[2].height, max(rect5[3].height, rect5[4].height))));
       //if (rect5[0].tl().x - big_left_ratio * max_height > 0 && (rect5[0].tl().y + big_down_ratio * max_height)<img.rows && (rect5[0].tl().x - big_left_ratio* max_height + big_width_ratio * max_height)<img.cols && (rect5[0].tl().y + big_down_ratio * max_height + big_height_ratio * max_height)<img.rows)
       //{
       ////ROI需要加一些约束
       //Mat rois = gray(Rect(int(rect5[0].tl().x - big_left_ratio * max_height), int(rect5[0].tl().y + big_down_ratio* max_height), int(big_width_ratio * max_height), int(big_height_ratio * max_height)));


       //----------------------------------------------直接到图像底面-------------------------------------------------------------------------
       float max_height = 1.2*max(rect5[0].height, max(rect5[1].height, max(rect5[2].height, max(rect5[3].height, rect5[4].height))));
       if ((rect5[0].tl().x - big_left_ratio * max_height) > 0 &&
           (rect5[0].tl().y + big_down_ratio * max_height) < img.rows &&
           (rect5[0].tl().x - big_left_ratio * max_height + big_width_ratio * max_height) < img.cols) {
           //---------ROI需要加一些约束---------
           Mat rois = gray(Rect(int(rect5[0].tl().x - big_left_ratio * max_height),
                                int(rect5[0].tl().y + big_down_ratio * max_height),
                                int(big_width_ratio * max_height),
                                int(img.rows - rect5[0].tl().y - big_down_ratio * max_height)));

           //---------------------------------------火焰数字查找数码管
           Mat roi_binary, roi_dilated;
           threshold(rois, roi_binary, big_threshold_value, 255, CV_THRESH_BINARY);
           dilate(roi_binary, roi_dilated, Mat());
           //dilate(roi_dilated, roi_dilated, Mat());
           // imshow("big_roi_dialted", roi_dilated);
           // imshow("big_roi_binary", roi_binary);
           vector<vector<Point> > rect_contours;
           findContours(roi_dilated, rect_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

           for (i = 0; i < rect_contours.size(); i++) {
               Rect poly = boundingRect(rect_contours[i]);
               double area = contourArea(rect_contours[i]);
               double hw_ratio = (double) poly.height / (double)poly.width;
               //cout<<"area:"<<area<<endl;
               //cout<<"hw_ratio:"<<hw_ratio<<endl;
               //cout<<"poly.height"<<poly.height<<endl;
               //----------面积大致在650--1400   比例大致在0.9-2.2      参数范围应该大一点
               if (area < 5000 && area > 500 && hw_ratio > 0.8 && hw_ratio <= 2.1 && poly.height > 20&&(poly.tl().x>2&&poly.br().x<(roi_binary.cols-2))) {
                   rect9.push_back(poly);
                   //cout<<"火焰数字九宫格数据：area:  "<<area<<"   hw_ratio:"<<hw_ratio<<endl;
               }
           }


           if (rect9.size() >= 9) {
               //----------如果火焰数字的九宫格大于九个，根据矩形的高度排序进行筛选------------
               if (rect9.size() > 9) {
                   for (i = 0; i < rect9.size() - 1; i++) {
                       int k = i;
                       for (int j = i + 1; j < rect9.size(); j++) {
                           if (rect9[k].height < rect9[j].height)
                               k = j;
                       }
                       if (k != i) {
                           Rect exc = rect9[i];
                           rect9[i] = rect9[k];
                           rect9[k] = exc;
                       }
                   }
                   while (rect9.size() > 9) {
                       rect9.erase(rect9.begin() + 9);
                   }
               }

               //---九宫格排序----------------------
               //cout<<"rect9.size:"<<rect9.size()<<endl;
               sort_rect(rect9);
               // //--------------------画出火焰数字的矩形框---------------------
               // string num_word[9]={"11","22","33","44","55","66","77","88","99"};
               // for (i = 0; i < 9; i++) {
               //     rectangle(roi_dilated, rect9[i].tl(), rect9[i].br(), Scalar(255), 1, 8);
               //     Point word;
               //     word.x = rect9[i].tl().x + 50;
               //     word.y = rect9[i].tl().y + 50;
               //     //cout<<"i:"<<i<<"   num_word[i]:"<<num_word[i]<<endl;
               //     putText(roi_dilated,num_word[i], word, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255));
               // }
               // //---------------------画出火焰数字矩形框的二值图--------------------------------
               // imshow("big_roi_rect:", roi_dilated);

               //检测密码是否变化
               if (codechange(now_code, pre_code)) {
                   goal_index = 0;
                   cout << "密码变化！" << endl;
               }

                //密码更新
                for (i = 0; i < 5; i++) {
                    pre_code[i] = now_code[i];
                }

                int goal = now_code[goal_index];
                index_rec[record_index] = hit_big_index(roi_binary, rect9, goal);
                record_index++;
                if (record_index == dep_num) {
                    record_index = record_index % dep_num;
                    goal_index++;
                    if (goal_index == 5)
                        goal_index = goal_index % 5;

                    int best_record;
                    best_record = choose_best(index_rec);


                    float x_offset=rect5[0].tl().x -  big_left_ratio * max_height;
                    float y_offset=rect5[0].tl().y +  big_down_ratio * max_height;


            vector<Point2f> points2d;
            vector<Point3f> points3d;


            points2d.clear();
            points2d.push_back(Point2f((rect9[0].tl().x+rect9[0].br().x)/2+x_offset,(rect9[0].tl().y+rect9[0].br().y)/2+y_offset));
            points2d.push_back(Point2f((rect9[2].tl().x+rect9[2].br().x)/2+x_offset,(rect9[2].tl().y+rect9[2].br().y)/2+y_offset));
            points2d.push_back(Point2f((rect9[8].tl().x+rect9[8].br().x)/2+x_offset,(rect9[8].tl().y+rect9[8].br().y)/2+y_offset));
            points2d.push_back(Point2f((rect9[6].tl().x+rect9[6].br().x)/2+x_offset,(rect9[6].tl().y+rect9[6].br().y)/2+y_offset));

            points3d.clear();
            points3d.push_back(Point3f(-b_width/2, -b_height/2, 0));
            points3d.push_back(Point3f(b_width/2, -b_height/2, 0));
            points3d.push_back(Point3f(b_width/2, b_height/2, 0));
            points3d.push_back(Point3f(-b_width/2, b_height/2, 0));

            Mat r, trans;
            solvePnP(points3d, points2d, intrinsic_Matrix, distortion_coefficients, r, trans);

            double *my_xyz = (double *)trans.data;
            double distance=my_xyz[2];

            distance=distance/10;
            int16_t xangle=0.0;
            int16_t yangle=0.0;
            if(best_record==6||best_record==7||best_record==8)
            {
                yangle=(-0.4*distance+120);
            }
            else if(best_record==0||best_record==1||best_record==2)
            {
                yangle=(-1)*((-0.4)*distance+120);
            }
            if(best_record==0||best_record==3||best_record==6)
            {
              xangle=(-1)*((-0.97)*distance+260);
            }
            else if(best_record==2||best_record==5||best_record==8)
            {
              xangle=(-0.97)*distance+260;
            }

            buff_msg.enemy_dist = distance;
            buff_msg.enemy_yaw = xangle;
            buff_msg.enemy_pitch = yangle;
            buff_msg.mode=flag_mode;

            //可以传送数据
            if_send = true;

				}
			}
			else
			{
				cout << "九宫格个数不对！" << endl;
			}
		}
		else
		{
			cout << "九宫格边框大小有误！" << endl;
		}
	}
}
//--------------------------------------------------------------------------



//----------------------------------------------------------------

//-------------自动打击函数-----------------------------------------
//-------------自动打击   函数  全局变量----------------------------------
//-------------自动打击   函数  全局变量----------------------------------
//------------------------自动打击  全局变量----------------------
#define color 1//1red 0blue
#define pi 3.14
#define match_num 5
#define height_threshold 0
#define distance_single_value 10
#define distance_multiple_value 10
#define drift_angle 18
#define a_r_final 3.3
#define a_r_b_max 5.5
#define a_r_b_min 4.1
#define a_r_s_max 3.5
#define a_r_s_min 1.3
#define o_r_max 2
#define o_r_min 0.5
#define x_multiple 2.2
#define y_multiple 3.5
#define r_r_max 2
#define r_r_min 0.5
#define similar_degree 0.85
#define area_value_min 50//40 2.3m 50 2.0m 70 1.7m
#define color_threshold 160
#define blueratio 0.2
#define blueratio2 0.2
#define redratio 0.3
#define redratio2 0.25
#define anglediff1 6
#define anglediff2 7
#define limit 1
#define horizontal_angle_thres 5

Mat model, element1, element2;
Point2f center, store_tem;
Rect external_rect, rect1, rect2, rect3, rect4, rect5, rrr, rect6, rect7, rect8, rect9, box;
float distancee, area;
int flag = 0;
int detection_flag=0;

float Max, a_r, o_r, r_r, angle_first, angle_second;
float horizontal_angle;
vector<Point> points1, points2, points3, points4, points5;

Mat Matrix=(Mat_<double>(3,3)<<501.9406,0.0,330.2514,0.0,502.2422,234.5579,0.0,0.0,1.0);
Mat coefficients=(Mat_<double>(5,1)<<0.0266,-0.0613,0.0,0.0,-0.0015);
// int stateNum = 2;
// int measureNum = 1;
// KalmanFilter KF(stateNum, measureNum, 0);
// //Mat processNoise(stateNum, 1, CV_32F);
// Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
// //KF.transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,//A 状态转移矩阵
// //        0, 1, 0, 1,
// //        0, 0, 1, 0,
// //        0, 0, 0, 1);
// KF.transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 1, 0, 1);
// //这里没有设置控制矩阵B，默认为零
// setIdentity(KF.measurementMatrix);//H=[1,0,0,0;0,1,0,0] 测量矩阵
// setIdentity(KF.processNoiseCov, Scalar::all(1e-5));//Q高斯白噪声，单位阵
// setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));//R高斯白噪声，单位阵
// setIdentity(KF.errorCovPost, Scalar::all(1));//P后验误差估计协方差矩阵，初始化为单位阵
// randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));



float juli(Point aa, Point bb)
{
    double sd;
    sd = sqrt((aa.x - bb.x)*(aa.x - bb.x) + (aa.y - bb.y)*(aa.y - bb.y));
    return sd;
}

double colorRatioR(Mat pic1, Mat pic2)
{
    double sum1 = 0, sum2 = 0;
    double ratio;
    for (int t = 0; t < pic1.rows; t++)
    {
        sum1 = sum1 + pic1.at<Vec3b>(t, pic1.cols/2)[2] / 255;
    }
    for (int t = 0; t < pic2.rows; t++)
    {
        sum2 = sum2 + pic2.at<Vec3b>(t, pic2.cols/2)[2] / 255;
    }
    ratio = (sum1 / (pic1.rows) + sum2 / (pic2.rows) ) / 2;
    return ratio;
}

double colorRatioB(Mat pic1, Mat pic2)
{
    double sum1 = 0, sum2 = 0;
    double ratio;
    for (int t = 0; t < pic1.rows; t++)
    {
        sum1 = sum1 + pic1.at<Vec3b>(t, pic1.cols/2)[0] / 255;
    }
    for (int t = 0; t < pic2.rows; t++)
    {
        sum2 = sum2 + pic2.at<Vec3b>(t, pic2.cols/2)[0] / 255;
    }
    ratio = (sum1 / (pic1.rows) + sum2 / (pic2.rows)) / 2;
    return ratio;
}

double colorRatioHSV(Mat pic1, Mat pic2)
{
    Mat element = getStructuringElement(MORPH_RECT, Size(1,7));
    erode(pic1,pic1,element);
    erode(pic2,pic2,element);
    cvtColor(pic1,pic1,CV_BGR2HSV);
    cvtColor(pic2,pic2,CV_BGR2HSV);
    double sum1 = 0, sum2 = 0;
    double ratio;
    for (int t = 0; t < pic1.rows; t++)
    {
        for (int tt = 0; tt < pic1.cols; tt++)
        {
            sum1 = sum1 + pic1.at<Vec3b>(t, tt)[0];
        }
    }
    for (int t = 0; t < pic2.rows; t++)
    {
        for (int tt = 0; tt < pic2.cols; tt++)
        {
            sum2 = sum2 + pic2.at<Vec3b>(t, tt)[0];
        }
    }
    ratio = (sum1 / (pic1.rows*pic1.cols) + sum2 / (pic2.rows*pic2.cols) ) / 2;
    return ratio;
}

double colorRatioHSV_S(Mat pic1, Mat pic2)
{
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	erode(pic1, pic1, element);
	erode(pic2, pic2, element);
	cvtColor(pic1, pic1, CV_BGR2HSV);
	cvtColor(pic2, pic2, CV_BGR2HSV);
	double sum1 = 0, sum2 = 0;
	double ratio;
	for (int t = 0; t < pic1.rows; t++)
	{
		for (int tt = 0; tt < pic1.cols; tt++)
		{
			sum1 = sum1 + pic1.at<Vec3b>(t, tt)[1];
		}
	}
	for (int t = 0; t < pic2.rows; t++)
	{
		for (int tt = 0; tt < pic2.cols; tt++)
		{
			sum2 = sum2 + pic2.at<Vec3b>(t, tt)[1];
		}
	}
	ratio = (sum1 / (pic1.rows*pic1.cols) + sum2 / (pic2.rows*pic2.cols)) / 2;
	return ratio;
}

Mat pretreat(Mat picture)
{
	Mat out;
	cvtColor(picture, out, CV_BGR2GRAY);
	Mat element1, element2;
	threshold(out, out, color_threshold, 255, CV_THRESH_BINARY);
	element1 = getStructuringElement(MORPH_RECT, Size(1, 5));
	erode(out, out, element1);
	element2 = getStructuringElement(MORPH_RECT, Size(3, 7));
	dilate(out, out, element2);
    //imshow("out",out);
	return out;
}

double hor_angle(Point p1, Point p2)
{
	int a, b;
	double rad, angle;
	a = abs(p1.x - p2.x);
	b = abs(p1.y - p2.y);
	if (a != 0)//ע��a!=0!!!
	{
		rad = atan(b / a);
		angle = rad * 180 / pi;
	}
	else
		angle = 90;
	return angle;
};


int auto_shot(Mat frame)
{
    Mat temp_frame = frame.clone();
    Mat src=pretreat(frame);
    vector<vector<Point> > contours;
    findContours(src, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    vector<RotatedRect> ellipsee(contours.size());
    int array1[1000];
    int count1 = 0;
    for (int i0 = 0; i0 < contours.size(); i0++)
    {
        if (contours[i0].size() < 5)
            continue;
        ellipsee[i0] = fitEllipse(contours[i0]);
        external_rect = ellipsee[i0].boundingRect();
        area = external_rect.area();
        if (area>30&&external_rect.height>height_threshold && (double)external_rect.height / (double)external_rect.width>1.8 && (ellipsee[i0].angle<drift_angle || ellipsee[i0].angle>(180 - drift_angle)))
        {
            points1.push_back(Point(ellipsee[i0].center));
            array1[count1] = i0;
            count1++;
	    if(count1>999)
	    {break;}
        }
    }
    int count2 = 0;
    int array2[2000];
    for (int i1 = 0; i1 < count1; i1++)
    {
        for (int j1 = i1 + 1; j1 < count1; j1++)
        {
            horizontal_angle = hor_angle(points1[i1], points1[j1]);
            if (horizontal_angle < horizontal_angle_thres)
            {
                points2.push_back(points1[i1]);
                points2.push_back(points1[j1]);
                array2[2 * count2] = array1[i1];
                array2[2 * count2 + 1] = array1[j1];
                count2++;
		        if(count2>999)
	                {break;}
            }
        }
		if(count2>999)
	        {break;}
    }
    int array3[1000];
    int count3 = 0;
    for (int i2 = 0; i2 < count2; i2++)
    {
        int label = 0;
        rect1 = boundingRect(contours[array2[i2 * 2]]);
  	rect2 = boundingRect(contours[array2[i2 * 2 + 1]]);
        rect3 = rect1 | rect2;
        for (int ii = 0; ii < points1.size(); ii++)
            label = label + rect3.contains(points1[ii]);
        if (label == 2)
        {
            points3.push_back(points2[i2 * 2]);
            points3.push_back(points2[i2 * 2 + 1]);
            array3[count3 * 2] = array2[i2 * 2];
            array3[count3 * 2 + 1] = array2[i2 * 2 + 1];
            count3++;
	        if(count3>499)
	        {break;}
        }
        else
            label = 0;
    }
    int array4[1000];
	int count4 = 0;
	for (int i3 = 0; i3 < count3; i3++)
	{
		rect4 = boundingRect(contours[array3[i3 * 2]]);
		rect5 = boundingRect(contours[array3[i3 * 2 + 1]]);
		rrr = rect4 | rect5;
		Max = max((double)rect4.height, (double)rect5.height);
		a_r = rrr.width / Max;
		o_r = (double)rect4.height / (double)rect5.height;
		r_r = ((double)rect4.height / (double)rect4.width) / ((double)rect5.height / (double)rect5.width);
		angle_first = ellipsee[array3[i3 * 2]].angle;
		angle_second = ellipsee[array3[i3 * 2 + 1]].angle;
		//rectangle(frame, rrr, 255, 1, 1);
		double s = colorRatioHSV_S(frame(rect4), frame(rect5));
		//double hsv = colorRatioHSV(frame(rect4), frame(rect5));
		double red = colorRatioR(temp_frame(rect4), temp_frame(rect5));
		double blue = colorRatioB(temp_frame(rect4), temp_frame(rect5));
		if (color == 1&&s>43)
		{
			if (a_r < a_r_s_max && a_r > a_r_s_min)
			{
				if (red > blue && red > redratio && blue < limit && (abs(angle_first - angle_second) < anglediff1 || abs(angle_first - angle_second) > (180 - anglediff1))
					&& rrr.height > height_threshold && o_r < o_r_max && o_r > o_r_min && r_r < r_r_max && r_r > r_r_min && (rect4.area()>area_value_min || rect5.area()>area_value_min))
				{
					//cout << a_r <<"  1"<<endl;
					//cout<<red<<"  "<<blue<<"  "<<white<<endl;
					points4.push_back(points3[i3 * 2]);
         			points4.push_back(points3[i3 * 2 + 1]);
					array4[2 * count4] = array3[i3 * 2];
					array4[2 * count4 + 1] = array3[i3 * 2 + 1];
					count4++;
				}
			}
			if (a_r > a_r_b_min && a_r < a_r_b_max)
			{
				if (red > blue && red > redratio2 && blue < limit && (abs(angle_first - angle_second) < anglediff2 || abs(angle_first - angle_second) > (180 - anglediff2))
					&& rrr.height > height_threshold && o_r < o_r_max && o_r > o_r_min && r_r < r_r_max && r_r > r_r_min)
				{
					//cout << a_r << "  2" << endl;
					//cout<<red<<"  "<<blue<<"  "<<white<<endl;
					points4.push_back(points3[i3 * 2]);
					points4.push_back(points3[i3 * 2 + 1]);
					array4[2 * count4] = array3[i3 * 2];
					array4[2 * count4 + 1] = array3[i3 * 2 + 1];
					count4++;
				}
			}
		}
		if (color == 0)
		{
		    //cout << a_r << endl;
			if (a_r < a_r_s_max && a_r > a_r_s_min)
			{
				if (((red < blue && blue > blueratio) || (red == blue)) && (abs(angle_first - angle_second) < anglediff1 || abs(angle_first - angle_second) > (180 - anglediff1))
					&& rrr.height > height_threshold && o_r < o_r_max && o_r > o_r_min && r_r < r_r_max && r_r > r_r_min && (rect4.area()>area_value_min || rect5.area()>area_value_min))
				{
					//cout<<red<<"  "<<blue<<"  "<<white<<endl;
					points4.push_back(points3[i3 * 2]);
					points4.push_back(points3[i3 * 2 + 1]);
					array4[2 * count4] = array3[i3 * 2];
					array4[2 * count4 + 1] = array3[i3 * 2 + 1];
					count4++;
				}
			}
			if (a_r > a_r_s_max && a_r < a_r_b_max)
	    	{
				if (red < blue && blue > blueratio2 && (abs(angle_first - angle_second) < anglediff2 || abs(angle_first - angle_second) > (180 - anglediff2))
					&& rrr.height > height_threshold && o_r < o_r_max && o_r > o_r_min && r_r < r_r_max && r_r > r_r_min)
				{
					//cout<<red<<"  "<<blue<<"  "<<white<<endl;
					points4.push_back(points3[i3 * 2]);
					points4.push_back(points3[i3 * 2 + 1]);
					array4[2 * count4] = array3[i3 * 2];
					array4[2 * count4 + 1] = array3[i3 * 2 + 1];
					count4++;
				}
			}
		}
	}

    int sizer = 3;//1big 0small
    int mark = 1;
    int markk = 1;
    Point point_temp1, point_temp2, temp;


    Mat trans = (Mat_<float>(3, 1));
    Mat r;
    Mat_<float> zuo;
    vector<cv::Point3f> point3d_da;
    vector<cv::Point2f> point2d_da;
    Point3f daji_3d;
    point3d_da.push_back(Point3f(-6.5, 3, 0));
    point3d_da.push_back(Point3f(6.5, 3, 0));
    point3d_da.push_back(Point3f(6.5, -3, 0));
    point3d_da.push_back(Point3f(-6.5, -3, 0));
    Point zs, ys, yx, zx;
    if (count4 > 1)
    {
        int aa, bb;
        int cc = 100, dd = 100;
        if (flag == 1)
        {
            for (int i4 = 0; i4 < points4.size() / 2; i4++)
            {
                point_temp1.x = points4[2 * i4].x;
                point_temp2.x = points4[2 * i4 + 1].x;
                point_temp1.y = points4[2 * i4].y;
                point_temp2.y = points4[2 * i4 + 1].y;
                temp.x = (point_temp1.x + point_temp2.x) / 2;
                temp.y = (point_temp1.y + point_temp2.y) / 2;
                distancee = juli(store_tem, temp);
                if (distancee < distance_multiple_value)
                {
                    aa = i4 * 2;
                    bb = i4 * 2 + 1;
                    cc = array4[i4 * 2];
                    dd = array4[i4 * 2 + 1];
                }
            }
            if (cc*dd<9999)
            {
                rect6 = boundingRect(contours[cc]);
                rect7 = boundingRect(contours[dd]);
                rect8 = rect6 | rect7;

                zs = rect8.tl();
                yx = rect8.br();
                ys = zs + Point(rect8.width,0);
                zx = zs + Point(0,rect8.height);
                point2d_da.push_back(zs);
                point2d_da.push_back(ys);
                point2d_da.push_back(yx);
                point2d_da.push_back(zx);
                solvePnP(point3d_da, point2d_da, Matrix, coefficients, r, trans);
                trans.convertTo(zuo, CV_32F);
                daji_3d.z = zuo.at<float>(0, 2);

                rect9.x = rect8.x - ((x_multiple - 1) / 2)*rect8.width;
                rect9.y = rect8.y - ((y_multiple - 1) / 2)*rect8.height;
                rect9.width = rect8.width * x_multiple;
                rect9.height = rect8.height * y_multiple;
                if (rect8.height>height_threshold && rect8.x > 0 && rect8.x + rect8.width < 640 && rect8.y > 0 && rect8.y + rect8.height < 480)
                    model = src(rect8);
                center.x = (points4[aa].x + points4[bb].x) / 2;
          	    center.y = (points4[aa].y + points4[bb].y) / 2;
                flag = 1;
                mark = 1;
		        if(rect8.width/rect8.height>a_r_final)
                    {sizer = 1;}
                if(rect8.width/rect8.height<a_r_final)
                    {sizer = 0;}
            }
            if (cc*dd>9999 && rect9.area() != 0 && rect8.area() != 0 && rect9.x > 0 && rect9.x + rect9.width<640 && rect9.y>0 && rect9.y + rect9.height < 480)
            {
                Mat similarity;
                matchTemplate(src(rect9), model, similarity, match_num);
                double mag_r;
                Point point;
                minMaxLoc(similarity, 0, &mag_r, 0, &point);
                if (model.cols > height_threshold && mag_r > similar_degree)
                {
                    box.x = point.x + rect9.x;
                    box.y = point.y + rect9.y;
                    box.width = rect8.width;
                    box.height = rect8.height;
                    center.x = box.x + box.width / 2;
                    center.y = box.y + box.height / 2;
                    //rectangle(frame, box, Scalar(0, 0, 255), 2);
                    model = src(box);
                    rect9.x = box.x - ((x_multiple - 1) / 2)*box.width;
                    rect9.y = box.y - ((y_multiple - 1) / 2)*box.height;
                    rect9.width = box.width * x_multiple;
                    rect9.height = box.height * y_multiple;
                    flag = 1;
                    mark = 1;

                    zs = box.tl();
                    yx = box.br();
                    ys = zs + Point(box.width, 0);
                    zx = zs + Point(0, box.height);
                    point2d_da.push_back(zs);
                    point2d_da.push_back(ys);
                    point2d_da.push_back(yx);
                    point2d_da.push_back(zx);
                    solvePnP(point3d_da, point2d_da, Matrix, coefficients, r, trans);
                    trans.convertTo(zuo, CV_32F);
                    daji_3d.z = zuo.at<float>(0, 2);
		                if(box.width/box.height>a_r_final)
                        {sizer = 1;}
                    if(box.width/box.height<a_r_final)
                        {sizer = 0;}
                }
                if (mag_r < similar_degree)
                {
                    flag = 0;
                    mark = 0;
                }
            }
        }
        if (flag == 0 && mark == 1)
        {
            rect6 = boundingRect(contours[array4[0]]);
            rect7 = boundingRect(contours[array4[1]]);
            rect8 = rect6 | rect7;

            zs = rect8.tl();
            yx = rect8.br();
            ys = zs + Point(rect8.width, 0);
            zx = zs + Point(0, rect8.height);
            point2d_da.push_back(zs);
            point2d_da.push_back(ys);
            point2d_da.push_back(yx);
            point2d_da.push_back(zx);
            solvePnP(point3d_da, point2d_da, Matrix, coefficients, r, trans);
            trans.convertTo(zuo, CV_32F);
            daji_3d.z = zuo.at<float>(0, 2);

            rect9.x = rect8.x - ((x_multiple - 1) / 2)*rect8.width;
            rect9.y = rect8.y - ((y_multiple - 1) / 2)*rect8.height;
            rect9.width = rect8.width * x_multiple;
            rect9.height = rect8.height * y_multiple;
            if (rect8.height > height_threshold && rect8.x > 0 && rect8.x + rect8.width < 640 && rect8.y > 0 && rect8.y + rect8.height < 480)
                model = src(rect8);
            center.x = (points4[0].x + points4[1].x) / 2;
      	    center.y = (points4[0].y + points4[1].y) / 2;
	          if(rect8.width/rect8.height>a_r_final)
                {sizer = 1;}
            if(rect8.width/rect8.height<a_r_final)
                {sizer = 0;}
            flag = 1;
        }
    }
    if (count4 == 1)
    {
        if (flag == 1)
        {
            center.x = (points4[0].x + points4[1].x) / 2;
            center.y = (points4[0].y + points4[1].y) / 2;
            distancee = juli(center, store_tem);
            if (distancee < distance_single_value)
            {
                rect6 = boundingRect(contours[array4[0]]);
                rect7 = boundingRect(contours[array4[1]]);
                rect8 = rect6 | rect7;

                zs = rect8.tl();
                yx = rect8.br();
                ys = zs + Point(rect8.width, 0);
                zx = zs + Point(0, rect8.height);
                point2d_da.push_back(zs);
                point2d_da.push_back(ys);
                point2d_da.push_back(yx);
                point2d_da.push_back(zx);
                solvePnP(point3d_da, point2d_da, Matrix, coefficients, r, trans);
                trans.convertTo(zuo, CV_32F);
                daji_3d.z = zuo.at<float>(0, 2);

                rect9.x = rect8.x - ((x_multiple - 1) / 2)*rect8.width;
                rect9.y = rect8.y - ((y_multiple - 1) / 2)*rect8.height;
                rect9.width = rect8.width * x_multiple;
                rect9.height = rect8.height * y_multiple;
                if (rect8.height > height_threshold && rect8.x > 0 && rect8.x + rect8.width < 640 && rect8.y > 0 && rect8.y + rect8.height < 480)
                    model = src(rect8);
		            if(rect8.width/rect8.height>a_r_final)
                    {sizer = 1;}
                if(rect8.width/rect8.height<a_r_final)
                    {sizer = 0;}
                flag = 1;
                markk = 1;
            }
            if (distancee > distance_single_value)
            {
                if (rect9.area() != 0 && rect8.area() != 0 && rect9.x > 0 && rect9.x + rect9.width<640 && rect9.y>0 && rect9.y + rect9.height < 480)
                {
                    Mat similarity;
                    matchTemplate(src(rect9), model, similarity, match_num);
                    double mag_r;
                    Point point;
                    minMaxLoc(similarity, 0, &mag_r, 0, &point);
                    if (model.cols > height_threshold && mag_r > similar_degree)
                    {
                        box.x = point.x + rect9.x;
                        box.y = point.y + rect9.y;
                        box.width = rect8.width;
                        box.height = rect8.height;
                        center.x = box.x + box.width / 2;
                        center.y = box.y + box.height / 2;
                        rectangle(frame, box, Scalar(0, 0, 255), 2);
                        model = src(box);
                        rect9.x = box.x - ((x_multiple - 1) / 2)*box.width;
                        rect9.y = box.y - ((y_multiple - 1) / 2)*box.height;
                        rect9.width = box.width * x_multiple;
                        rect9.height = box.height * y_multiple;
                        flag = 1;
                        markk = 1;

                        zs = box.tl();
                        yx = box.br();
                        ys = zs + Point(box.width, 0);
                        zx = zs + Point(0, box.height);
                        point2d_da.push_back(zs);
                        point2d_da.push_back(ys);
                        point2d_da.push_back(yx);
                        point2d_da.push_back(zx);
                        solvePnP(point3d_da, point2d_da, Matrix, coefficients, r, trans);
                        trans.convertTo(zuo, CV_32F);
                        daji_3d.z = zuo.at<float>(0, 2);
			                  if(box.width/box.height>a_r_final)
                            {sizer = 1;}
                        if(box.width/box.height<a_r_final)
                            {sizer = 0;}
                    }
                    if (mag_r < similar_degree)
                    {
                        flag = 0;
                        markk = 0;
                    }
                }
            }
        }
        if (flag == 0 && markk == 1)
        {
            rect6 = boundingRect(contours[array4[0]]);
  		    rect7 = boundingRect(contours[array4[1]]);
            rect8 = rect6 | rect7;

            zs = rect8.tl();
            yx = rect8.br();
            ys = zs + Point(rect8.width, 0);
            zx = zs + Point(0, rect8.height);
            point2d_da.push_back(zs);
            point2d_da.push_back(ys);
            point2d_da.push_back(yx);
            point2d_da.push_back(zx);
            solvePnP(point3d_da, point2d_da, Matrix, coefficients, r, trans);
            trans.convertTo(zuo, CV_32F);
            daji_3d.z = zuo.at<float>(0, 2);

            rect9.x = rect8.x - ((x_multiple - 1) / 2)*rect8.width;
            rect9.y = rect8.y - ((y_multiple - 1) / 2)*rect8.height;
            rect9.width = rect8.width * x_multiple;
            rect9.height = rect8.height * y_multiple;
            if (rect8.height > height_threshold && rect8.x > 0 && rect8.x + rect8.width < 640 && rect8.y > 0 && rect8.y + rect8.height < 480)
                model = src(rect8);
            center.x = (points4[0].x + points4[1].x) / 2;
            center.y = (points4[0].y + points4[1].y) / 2;
            if(rect8.width/rect8.height>a_r_final)
                {sizer = 1;}
            if(rect8.width/rect8.height<a_r_final)
                {sizer = 0;}
            flag = 1;
        }
    }
    if (count4 < 1)
    {
        if (rect9.area() != 0 && rect8.area() != 0 && rect9.x > 0 && rect9.x + rect9.width<640 && rect9.y>0 && rect9.y + rect9.height < 480)
        {
            Mat similarity;
            matchTemplate(src(rect9), model, similarity, match_num);
            double mag_r;
            Point point;
            minMaxLoc(similarity, 0, &mag_r, 0, &point);
            if (model.cols > height_threshold && mag_r > similar_degree)
            {
                box.x = point.x + rect9.x;
                box.y = point.y + rect9.y;
                box.width = rect8.width;
                box.height = rect8.height;
                center.x = box.x + box.width / 2;
                center.y = box.y + box.height / 2;
                rectangle(frame, box, Scalar(0, 0, 255), 2);
                model = src(box);
                rect9.x = box.x - ((x_multiple - 1) / 2)*box.width;
                rect9.y = box.y - ((y_multiple - 1) / 2)*box.height;
                rect9.width = box.width * x_multiple;
                rect9.height = box.height * y_multiple;
                flag = 1;

                zs = box.tl();
                yx = box.br();
                ys = zs + Point(box.width, 0);
                zx = zs + Point(0, box.height);
                point2d_da.push_back(zs);
                point2d_da.push_back(ys);
                point2d_da.push_back(yx);
                point2d_da.push_back(zx);
                solvePnP(point3d_da, point2d_da, Matrix, coefficients, r, trans);
                trans.convertTo(zuo, CV_32F);
                daji_3d.z = zuo.at<float>(0, 2);
                if(box.width/box.height>a_r_final)
                {
                   sizer = 1;
                }
                if(box.width/box.height<a_r_final)
                    {sizer = 0;}
            }
            if (mag_r < similar_degree)
                flag = 0;
        }
    }


    if (center.x*center.y != 0 && sizer < 2)
    {
		flag = 1;
        circle(frame, center, 2, Scalar(255, 255, 255), 2, 2);

	    if(sizer == 1)
            {daji_3d.z = daji_3d.z * 2;}
        if(sizer == 0)
            {daji_3d.z = daji_3d.z * 1.2;}
        enemy_msg.enemy_dist = daji_3d.z;
        enemy_msg.enemy_yaw = center.x-320;
        enemy_msg.enemy_pitch = center.y-240;
	cout << center.x << "    " << center.y << endl;
	ROS_INFO("write[%d]",center.x);
	detection_flag=1;
        //enemy_msg.mode=sizer;
    }
    else
    {
	  flag = 0;
      center.x=1000;
      center.y=1000;
      daji_3d.z=1000;
      sizer=2;
      enemy_msg.enemy_dist = daji_3d.z;
      enemy_msg.enemy_yaw = center.x-320;
      enemy_msg.enemy_pitch = center.y-240;
      detection_flag=0;
      //enemy_msg.mode=sizer;
    }
    if_send=true;
    imshow("1", frame);
    waitKey(1);
    store_tem.x = center.x;
    store_tem.y = center.y;
    center.x = 0;
    center.y = 0;
    mark = 1;
    markk = 1;
    points1.clear();
    points2.clear();
    points3.clear();
    points4.clear();
    points5.clear();
    return 0;
}
//------------------------------------------------------------


int main_last()
{
        //------------------没有进入模式之前的常规显示操作----------------
		Mat image, resize_image;
        image = cv_ptr->image;
		if (!image.data)
		{
			cout << "can not read the picture!" << endl;
      return 0;
		}
		resize(image, resize_image, Size(640, 480));
        //imshow("resize_image",resize_image);
        //---录视频代码-----------------------------------------------
        //  if(record_num==0)
        //  {
        //   record_num=1000;
        //    time_t t = time(0);
        //   strftime(filename, sizeof(filename), "/home/xjturm/video_record/%Y-%m-%d %H-%M-%S", localtime(&t));
        //    strcat(filename, extendname);
        //    ROS_INFO("filename is %s",filename);
        //    cam.open(filename, CV_FOURCC('M', 'J', 'P', 'G'), 25, resize_image.size());
        // }
        //  else if(record_num>0)
        //  {
        //    cam.write(resize_image);
        //    record_num--;
        //  }
        //------------------------------------------------------------

        //----------------大小能量机关模式-------------------------------
  		if (flag_mode== 1||flag_mode== 2)
  		{
        if(if_first)
        {
          system(arg_buff);
          buff_msg.enemy_dist = 10;
          buff_msg.enemy_yaw = 0;
          buff_msg.enemy_pitch = 0;
          buff_msg.mode=flag_mode;
          //可以传输数据
          if_send=true;
          if_first=false;
          //结束本次进程
        }
        else
        {
         //-------------手写数字----------------------
         if(flag_mode == 2)
         {
           get_small_rect(image);
  		   }
          //-----------火焰数字-----------------------
  			 else if(flag_mode==1)
  			 {
           get_big_rect(image);
         }
        }
      }
      //---------------自动打击模式--------------------------------------------------
      else if(flag_mode==3)
      {

        if(if_first)
        {
          system(arg_armour);
          if_first=false;
        }
        auto_shot(image);
      }
      //-----------------------------------------------------------------
      //------------------退出大小符，自动打击模式----------------------------------------
      else if(flag_mode==4)
      {
        //----------大小能量机关的设置更新-----------------------------------
        system(arg_armour);
        now_code[5] = { 0 };
        pre_code[5] = { 0 };
        goal_index = 0;
        record_index = 0;
        index_rec[dep_num] = { 0 };
        if_send=false;
        if_first=true;
        //---------------------------------------------------------------
      }
      //--------------串口初始化------------------------------
      else if(flag_mode==0)
      {
        buff_msg.enemy_dist = 10;
        buff_msg.enemy_yaw = 60;
        buff_msg.enemy_pitch = 30;
        buff_msg.mode=flag_mode;

        if_send=true;
      }

return 0;
}

// //---------------------------------------------------------------------
// void read_callback(const serial_common::Infantrymode::ConstPtr& msg)
// {
//   flag_mode = msg-> mode;
//   ROS_INFO_STREAM("RECEIVE: "<< flag_mode);
// }
// //---------------------------------------------------------------------
//

//---------------------智能射击回调函数-----------------------------
//---------------------------------------------------------------------
void read_callback(const serial_common::Infantrymode::ConstPtr& msg)
{
  pre_mode=now_mode;
  flag_mode = msg-> mode;
  now_mode=flag_mode;
  if(pre_mode!=now_mode)
  {
    //----------大小能量机关的设置更新-----------------------------------
    now_code[5] = { 0 };
    pre_code[5] = { 0 };
    goal_index = 0;
    record_index = 0;
    index_rec[dep_num] = { 0 };
    if_send=false;
    if_first=true;
    system(arg_buff);
    ROS_INFO_STREAM("步兵mode发生变化！ ");
    //---------------------------------------------------------------
  }
  ROS_INFO_STREAM("RECEIVE: "<< flag_mode);
}
//---------------------------------------------------------------------

static const std::string OPENCV_WINDOW = "Image window node ";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::Publisher enemy_pub = nh_.advertise<serial_common::EnemyPos>("enemy", 1);
  ros::Publisher buff_pub = nh_.advertise<serial_common::EnemyPos>("buff", 1);
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

    if(if_send)
    {
      if(flag_mode==1||flag_mode==2||flag_mode==0)
      {
	      buff_pub.publish(buff_msg);
          if_send=false;
          flag_mode=10;//恢复默认flag_mode
          ROS_INFO_STREAM("buff_SEND: ");
      }
      else if(flag_mode==3)
      {
	//if(detection_flag==1)
	//{
        	enemy_pub.publish(enemy_msg);
        	ROS_INFO_STREAM("enemy_SEND: ");
		detection_flag=0;
	//}
      }
    }
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
