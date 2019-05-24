//-----------------------------------【头文件包含部分】---------------------------------------  
//      描述：包含程序所依赖的头文件
//----------------------------------------------------------------------------------------------  
#include <ros/ros.h>
#include <iostream> //C++标准输入输出库  
#include <image_transport/image_transport.h> /*image_transport 头文件用来在ROS系统中的话题上发布和订阅图象消息 */ 
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h> /* ROS图象类型的编码函数 */ 
//#include <opencv2/core/core.hpp> //OpenCV2标准头文件   
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>  
#include <iostream> 
#include <stdio.h>
#include <string>
#include <math.h>//对应下面的pow（平方）

#include "std_msgs/String.h"

#include <sstream>
/////////////////////////////////////双灯视觉定位程序/////////////////////////////////////////////////////////

#define pi 3.1415926

static const std::string OUTPUT = "Output"; //定义输出窗口名称


//-----------------------------------【命名空间声明部分】--------------------------------------  
//      描述：包含程序所使用的命名空间  
//-----------------------------------------------------------------------------------------------  
using namespace cv;
using namespace std;

//----------------------------------·【结构体】--------------------------------------------
//      描述：定义各种结构体  
//----------------------------------------------------------------------------------------------- 
struct XYZ{	//坐标处理函数的结构体，用于放置坐标值
	double x;
	double y;
	double z;
	Mat img_point;
	};

struct LED{	// LED处理过程的结构体，用于存放图像处理过程中的信息以及处理结果
	int ID;								//	ID,条纹数目
	double Img_local_X, Img_local_Y;	// LED在图像上的像素坐标位置，,x坐标,y坐标
	double X, Y; 						// LED灯具的真实位置,x坐标,y坐标
	Mat img_next, matBinary;			
	int X_min, X_max, Y_min, Y_max;
	Mat image_cut;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	int num;
	};
	
struct position{// LED的位置，对应不同位置的灯具
	int max;	// ID_max,最大条纹数目 	
	int min;	// ID_min，最小条纹数目
	double X;	// LED灯具的真实位置,x坐标
	double Y;	// LED灯具的真实位置,y坐标
	};

struct XYZ pose_value;
Mat img_point;
//-----------------------------------------------------------------------------------------------
//**********************************************************************************************
//
//      *********************             【函数声明部分】              *******************
//
//**********************************************************************************************
//-----------------------------------------------------------------------------------------------


//-----------------------------------【threshold自动阈值】-----------------------------------------  
//      描述：OpenCV中threshold自动阈值，类似matlab中的graythresh。为了二值化
//-----------------------------------------------------------------------------------------------  
double getThreshVal_Otsu_8u(const cv::Mat& _src)
{
	cv::Size size = _src.size();
	if (_src.isContinuous())
	{
		size.width *= size.height;
		size.height = 1;
	}
	const int N = 256;
	int i, j, h[N] = { 0 };
	for (i = 0; i < size.height; i++)
	{
		const uchar* src = _src.data + _src.step*i;
		for (j = 0; j <= size.width - 4; j += 4)
		{
			int v0 = src[j], v1 = src[j + 1];
			h[v0]++; h[v1]++;
			v0 = src[j + 2]; v1 = src[j + 3];
			h[v0]++; h[v1]++;
		}
		for (; j < size.width; j++)
			h[src[j]]++;
	}

	double mu = 0, scale = 1. / (size.width*size.height);
	for (i = 0; i < N; i++)
		mu += i*h[i];

	mu *= scale;
	double mu1 = 0, q1 = 0;
	double max_sigma = 0, max_val = 0;

	for (i = 0; i < N; i++)
	{
		double p_i, q2, mu2, sigma;

		p_i = h[i] * scale;
		mu1 *= q1;
		q1 += p_i;
		q2 = 1. - q1;

		if (std::min(q1, q2) < FLT_EPSILON || std::max(q1, q2) > 1. - FLT_EPSILON)
			continue;

		mu1 = (mu1 + i*p_i) / q1;
		mu2 = (mu - q1*mu1) / q2;
		sigma = q1*q2*(mu1 - mu2)*(mu1 - mu2);
		if (sigma > max_sigma)
		{
			max_sigma = sigma;
			max_val = i;
		}
	}

	return max_val;
}



//将图片中的LED逐个进行分割
void ls_LED(const Mat& _img, int& X_min, int& X_max, int& Y_min, int& Y_max, Mat& img_next)
{
	Mat temp1= _img.clone();
	
	//求xmin与xmax
	int row1 = temp1.rows;//行数
	int col1 = temp1.cols;//列
	int j = 0;//注意是从0开始
	while (j < col1)//j的初值为1
	{
		double sum1 = 0.0;
		for (int i = 0;i < row1;i++)//注意没有等于号
		{
			uchar* data1 = temp1.ptr<uchar>(i);//ptr<uchar>(i)[j]访问第i行第j列的像素
			sum1 = data1[j] + sum1;
		}//将第j列的每一行加完
		if (sum1>-0.000001 && sum1< 0.000001)//double类型，不能写==0
		{
			j++;
		}
		else
		{
			break;//跳出这个while循环
		}

	}
	X_min = j;

	while (j < col1)//j的初值为X_min 
	{
		double sum1 = 0.0;
		for (int i = 0;i < row1;i++)
		{
			uchar* data1 = temp1.ptr<uchar>(i);//ptr<uchar>(i)[j]访问第i行第j列的像素
			sum1 = data1[j] + sum1;
		}//将第j列的每一行XXXXXX加完
		if (sum1 != 0)
		{
			j++;
		}
		else
		{
			break;//跳出这个while循环
		}
	}
	X_max = j;

	//进行切割
	Mat image_cut = temp1(Rect(X_min, 0, X_max - X_min, row1));
	Mat temp = image_cut.clone();



	//求ymin与ymax
	int row = temp.rows;//行数
	int col = temp.cols;//列
	int i = 0;
	while (i < row)//i的初值为1
	{
		double sum = 0.0;
		uchar* data = temp.ptr<uchar>(i);
		for (j = 0;j < col;j++)//对每一行中的每一列像素进行相加，ptr<uchar>(i)[j]访问第i行第j列的像素
		{
			sum = data[j] + sum;
		}//最终获得第i行的列和
		if (sum>-0.000001 && sum < 0.000001)
		{
			i++;
		}
		else
		{
			Y_min = i;
			break;//跳出这个while循环
		}
	}
	Y_min = i;

	while (i <= row-16)//i的初值为Y_min
	{
		double sum = 0.0;
		uchar* data = temp.ptr<uchar>(i);
		for (j = 0;j < col;j++)//对每一行中的每一列像素进行相加，ptr<uchar>(i)[j]访问第i行第j列的像素
		{
			sum = data[j] + sum;
		}//最终获得第i行的列和
		if (sum != 0)
		{
			i++;
		}
		else
		{
			double sum6 = 0.0;
			int iiii = i + 16;
			uchar* data = temp.ptr<uchar>(iiii);
			for (j = 0;j < col;j++)//对每一行中的每一列像素进行相加，ptr<uchar>(i)[j]访问第i行第j列的像素
			{
				sum6 = data[j] + sum6;
			}//最终获得第i行之后20行，即iiii的列和
			if (sum6 > -0.000001 && sum6 < 0.000001)//如果仍然为0，才跳出
			{
				Y_max = i;
				goto logo;//跳出这个while循环
			}
			else//否则继续执行
			{
				i++;
			}
		}
	}
	logo:
	Y_max = i;

	//进行切割
	Mat image_cut1 = temp(Rect(0, Y_min, col, Y_max - Y_min));
	img_next = image_cut1.clone();   //clone函数创建新的图片 
}



//起到MATLAB中，bwareaopen的功能，去除连通区域少于n的部分
void bwareaopen(Mat &data, int n)
{
	Mat labels, stats, centroids;
	connectedComponentsWithStats(data, labels, stats, centroids, 8, CV_16U);
	int regions_count = stats.rows - 1;
	int regions_size, regions_x1, regions_y1, regions_x2, regions_y2;

	for (int i = 1;i <= regions_count;i++)
	{
		regions_size = stats.ptr<int>(i)[4];
		if (regions_size < n)
		{
			regions_x1 = stats.ptr<int>(i)[0];
			regions_y1 = stats.ptr<int>(i)[1];
			regions_x2 = regions_x1 + stats.ptr<int>(i)[2];
			regions_y2 = regions_y1 + stats.ptr<int>(i)[3];

			for (int j = regions_y1;j<regions_y2;j++)
			{
				for (int k = regions_x1;k<regions_x2;k++)
				{
					if (labels.ptr<ushort>(j)[k] == i)
						data.ptr<uchar>(j)[k] = 0;
				}
			}
		}
	}
}



//实现对图像的细化
void chao_thinimage(Mat &srcimage)//单通道、二值化后的图像  
{
	vector<Point> deletelist1;
	int Zhangmude[9];
	int nl = srcimage.rows;
	int nc = srcimage.cols;
	while (true)
	{
		for (int j = 1; j < (nl - 1); j++)
		{
			uchar* data_last = srcimage.ptr<uchar>(j - 1);
			uchar* data = srcimage.ptr<uchar>(j);
			uchar* data_next = srcimage.ptr<uchar>(j + 1);
			for (int i = 1; i < (nc - 1); i++)
			{
				if (data[i] == 255)
				{
					Zhangmude[0] = 1;
					if (data_last[i] == 255) Zhangmude[1] = 1;
					else  Zhangmude[1] = 0;
					if (data_last[i + 1] == 255) Zhangmude[2] = 1;
					else  Zhangmude[2] = 0;
					if (data[i + 1] == 255) Zhangmude[3] = 1;
					else  Zhangmude[3] = 0;
					if (data_next[i + 1] == 255) Zhangmude[4] = 1;
					else  Zhangmude[4] = 0;
					if (data_next[i] == 255) Zhangmude[5] = 1;
					else  Zhangmude[5] = 0;
					if (data_next[i - 1] == 255) Zhangmude[6] = 1;
					else  Zhangmude[6] = 0;
					if (data[i - 1] == 255) Zhangmude[7] = 1;
					else  Zhangmude[7] = 0;
					if (data_last[i - 1] == 255) Zhangmude[8] = 1;
					else  Zhangmude[8] = 0;
					int whitepointtotal = 0;
					for (int k = 1; k < 9; k++)
					{
						whitepointtotal = whitepointtotal + Zhangmude[k];
					}
					if ((whitepointtotal >= 2) && (whitepointtotal <= 6))
					{
						int ap = 0;
						if ((Zhangmude[1] == 0) && (Zhangmude[2] == 1)) ap++;
						if ((Zhangmude[2] == 0) && (Zhangmude[3] == 1)) ap++;
						if ((Zhangmude[3] == 0) && (Zhangmude[4] == 1)) ap++;
						if ((Zhangmude[4] == 0) && (Zhangmude[5] == 1)) ap++;
						if ((Zhangmude[5] == 0) && (Zhangmude[6] == 1)) ap++;
						if ((Zhangmude[6] == 0) && (Zhangmude[7] == 1)) ap++;
						if ((Zhangmude[7] == 0) && (Zhangmude[8] == 1)) ap++;
						if ((Zhangmude[8] == 0) && (Zhangmude[1] == 1)) ap++;
						if (ap == 1)
						{
							if ((Zhangmude[1] * Zhangmude[7] * Zhangmude[5] == 0) && (Zhangmude[3] * Zhangmude[5] * Zhangmude[7] == 0))
							{
								deletelist1.push_back(Point(i, j));
							}
						}
					}
				}
			}
		}
		if (deletelist1.size() == 0) break;
		for (size_t i = 0; i < deletelist1.size(); i++)
		{
			Point tem;
			tem = deletelist1[i];
			uchar* data = srcimage.ptr<uchar>(tem.y);
			data[tem.x] = 0;
		}
		deletelist1.clear();

		for (int j = 1; j < (nl - 1); j++)
		{
			uchar* data_last = srcimage.ptr<uchar>(j - 1);
			uchar* data = srcimage.ptr<uchar>(j);
			uchar* data_next = srcimage.ptr<uchar>(j + 1);
			for (int i = 1; i < (nc - 1); i++)
			{
				if (data[i] == 255)
				{
					Zhangmude[0] = 1;
					if (data_last[i] == 255) Zhangmude[1] = 1;
					else  Zhangmude[1] = 0;
					if (data_last[i + 1] == 255) Zhangmude[2] = 1;
					else  Zhangmude[2] = 0;
					if (data[i + 1] == 255) Zhangmude[3] = 1;
					else  Zhangmude[3] = 0;
					if (data_next[i + 1] == 255) Zhangmude[4] = 1;
					else  Zhangmude[4] = 0;
					if (data_next[i] == 255) Zhangmude[5] = 1;
					else  Zhangmude[5] = 0;
					if (data_next[i - 1] == 255) Zhangmude[6] = 1;
					else  Zhangmude[6] = 0;
					if (data[i - 1] == 255) Zhangmude[7] = 1;
					else  Zhangmude[7] = 0;
					if (data_last[i - 1] == 255) Zhangmude[8] = 1;
					else  Zhangmude[8] = 0;
					int whitepointtotal = 0;
					for (int k = 1; k < 9; k++)
					{
						whitepointtotal = whitepointtotal + Zhangmude[k];
					}
					if ((whitepointtotal >= 2) && (whitepointtotal <= 6))
					{
						int ap = 0;
						if ((Zhangmude[1] == 0) && (Zhangmude[2] == 1)) ap++;
						if ((Zhangmude[2] == 0) && (Zhangmude[3] == 1)) ap++;
						if ((Zhangmude[3] == 0) && (Zhangmude[4] == 1)) ap++;
						if ((Zhangmude[4] == 0) && (Zhangmude[5] == 1)) ap++;
						if ((Zhangmude[5] == 0) && (Zhangmude[6] == 1)) ap++;
						if ((Zhangmude[6] == 0) && (Zhangmude[7] == 1)) ap++;
						if ((Zhangmude[7] == 0) && (Zhangmude[8] == 1)) ap++;
						if ((Zhangmude[8] == 0) && (Zhangmude[1] == 1)) ap++;
						if (ap == 1)
						{
							if ((Zhangmude[1] * Zhangmude[3] * Zhangmude[5] == 0) && (Zhangmude[3] * Zhangmude[1] * Zhangmude[7] == 0))
							{
								deletelist1.push_back(Point(i, j));
							}
						}
					}
				}
			}
		}
		if (deletelist1.size() == 0) break;
		for (size_t i = 0; i < deletelist1.size(); i++)
		{
			Point tem;
			tem = deletelist1[i];
			uchar* data = srcimage.ptr<uchar>(tem.y);
			data[tem.x] = 0;
		}
		deletelist1.clear();
	}
}

struct XYZ double_LED(double f,double Center_X, double Center_Y, struct LED A,struct LED B)
{
	double ImgX1;
	double ImgY1;
	double ImgX2;
	double ImgY2;
	double x1;
	double y1;
	double x2;
	double y2;

	if (A.Y>B.Y){
		ImgX1 = A.Img_local_X;
		ImgY1 = A.Img_local_Y;
		ImgX2 = B.Img_local_X;
		ImgY2 = B.Img_local_Y;
		x1 = A.X;
		y1 = A.Y;
		x2 = B.X;
		y2 = B.Y;
	}

	else
	{
		ImgX1 = B.Img_local_X;
		ImgY1 = B.Img_local_Y;
		ImgX2 = A.Img_local_X;
		ImgY2 = A.Img_local_Y;
		x1 = B.X;
		y1 = B.Y;
		x2 = A.X;
		y2 = A.Y;
	}

	double alpha;

	if (x1>x2){
		alpha = -(3*pi/4);
	}

	else
	{
		alpha = -(pi/4);
	}
	cout << "alpha=" << alpha << '\n';


	double d_12 = sqrt(pow((ImgX1 - ImgX2),2) + pow((ImgY1 - ImgY2),2))*3.2e-3;
	double D_12 = sqrt(pow((x1 - x2),2) + pow((y1 - y2),2));
	double H = D_12 / d_12*f;
	double X_r = ((ImgX1 + ImgX2) / 2 - Center_X)*3.2e-3*H / f;
	double Y_r = ((ImgY1 + ImgY2) / 2 - Center_Y)*3.2e-3*H / f;
	double X_c = (x1 + x2) / 2;
	double Y_c = (y1 + y2) / 2;
	double X = X_r;
	double Y = Y_r;

	// cout << "H=" << H << '\n';

	// 计算角度
	// cout << "ImgY2=" << ImgY2 << '\n';
	// cout << "ImgY1=" << ImgY1 << '\n';
	// cout << "ImgX2 =" << ImgX2 << '\n';
	// cout << "ImgX1=" << ImgX1 << '\n';
	double K1 = abs((ImgY2 - ImgY1) / (ImgX2 - ImgX1));
	// cout << "K1=" << K1  << '\n';
	double angle = atan(K1);
	// cout << "angle1=" << angle / pi * 180 << '\n';

	//由于对称性，要对角度做进一步处理
	bool ABC = ImgY2 < ImgY1;
	bool EFG = ImgX2 > ImgX1;
	int ABCD = ABC * 2 + EFG;
	// ABCD = 3;
	// cout << "ABCD=" << ABCD << '\n';

	switch (ABCD)
	{
	case 0:
		angle = angle + alpha;
		break;
	case 1:
		angle = pi - angle + alpha;
		break;
	case 2:
		angle = 2 * pi - angle + alpha;
		break;
	case 3:
		angle = angle + pi + alpha;
		break;
	}
	// cout << "angle=" << angle / pi * 180 << '\n';
		
	double XX = X*cos(angle) - Y*sin(angle);
	double YY = X*sin(angle) + Y*cos(angle);

	XX = XX + X_c;
	YY = YY + Y_c;

	double xx = XX / 10;
	double yy = YY / 10;
	double zz = 150 - H / 10;

	// imshow("test time", grayImage);

	struct XYZ pose;
	pose.x=xx;
	pose.y=yy;
	pose.z=zz;

}

struct XYZ three_LED(double f, double Center_X, double Center_Y, struct LED A,struct LED B,struct LED C)
{
	double ImgX1 = A.Img_local_X;
	double ImgY1 = A.Img_local_Y;
	double ImgX2 = B.Img_local_X;
	double ImgY2 = B.Img_local_Y;
	double ImgX3 = C.Img_local_X;
	double ImgY3 = C.Img_local_Y;
	double x1 = A.X;
	double y1 = A.Y;
	double x2 = B.X;
	double y2 = B.Y;
	double x3 = C.X;
	double y3 = C.Y;

	//三灯定位
	double d_12 = sqrt(pow((ImgX1 - ImgX2), 2) + pow((ImgY1 - ImgY2), 2))*3.2e-3;
	double d_13 = sqrt(pow((ImgX1 - ImgX3), 2) + pow((ImgY1 - ImgY3), 2))*3.2e-3;
	double d_23 = sqrt(pow((ImgX2 - ImgX3), 2) + pow((ImgY2 - ImgY3), 2))*3.2e-3;
	double D_12 = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
	double D_13 = sqrt(pow((x1 - x3), 2) + pow((y1 - y3), 2));
	double D_23 = sqrt(pow((x2 - x3), 2) + pow((y2 - y3), 2));
	double H = (D_12 / d_12*f + D_13 / d_13*f + D_23 / d_23*f) / 3;//计算出高度
	// cout << "H=" << H << '\n';

	//计算水平方向上摄像头到3个LED的距离
	double d_1 = sqrt(pow((ImgX1 - Center_X), 2) + pow((ImgY1 - Center_Y), 2))*3.2e-3;
	double d_2 = sqrt(pow((ImgX2 - Center_X), 2) + pow((ImgY2 - Center_Y), 2))*3.2e-3;
	double d_3 = sqrt(pow((ImgX3 - Center_X), 2) + pow((ImgY3 - Center_Y), 2))*3.2e-3;

	//对应真实的距离
	double D_1 = H / f*d_1;
	double D_2 = H / f*d_2;
	double D_3 = H / f*d_3;

	double r1 = pow(D_1, 2);
	double r2 = pow(D_2, 2);
	double r3 = pow(D_3, 2);

	// double rr1 = pow(d_1, 2);
	// double rr2 = pow(d_2, 2);
	// double rr3 = pow(d_3, 2);

	//解出终端的位置坐标
	double a1 = 2 * (x1 - x3);
	double b1 = 2 * (y1 - y3);
	double c1 = pow(x3, 2) - pow(x1, 2) + pow(y3, 2) - pow(y1, 2) - r3 + r1;
	double a2 = 2 * (x2 - x3);
	double b2 = 2 * (y2 - y3);
	double c2 = pow(x3, 2) - pow(x2, 2) + pow(y3, 2) - pow(y2, 2) - r3 + r2;

	// double a1 = 2 * (ImgX1 - ImgX3);
	// double b1 = 2 * (ImgY1 - ImgY3);
	// double c1 = pow(ImgX3, 2) - pow(ImgX1, 2) + pow(ImgY3, 2) - pow(ImgY1, 2) - rr3 + rr1;
	// double a2 = 2 * (ImgX2 - ImgX3);
	// double b2 = 2 * (ImgY2 - ImgY3);
	// double c2 = pow(ImgX3, 2) - pow(ImgX2, 2) + pow(ImgY3, 2) - pow(ImgY2, 2) - rr3 + rr2;

	double XX = (c2 * b1 - c1 * b2) / (a1*b2 - a2 * b1);
	double YY = (c2 * a1 - c1 * a2) / (a2*b1 - a1 * b2);

	double xx = XX / 10;
	double yy = YY / 10;
	// xx = xx*(f / H);
	// yy = xx*(f / H);
	double zz = 150 - H / 10;

	struct XYZ pose;
	pose.x=xx;
	pose.y=yy;
	pose.z=zz;
	return pose;

}

//-----------------------------------【Get_coordinate()函数】------------------------------------
//      描述：双灯定位，灰度图像传入
//-----------------------------------------------------------------------------------------------  
struct XYZ Get_coordinate(cv::Mat img)
// int main()
// 1 2/3 4/5 6/7     9/10     11/12
{
	struct LED unkonwn,A,B,C,D,E,F;
	// cout << "111" << '\n';
struct position P1 = {	// LED 序号
		5,		// ID_max,最大条纹数目 
		4,		// ID_min，最小条纹数目
		470,	// LED灯具的真实位置,x坐标
		940,	// LED灯具的真实位置,y坐标
	};

	struct position P2 = {	// LED 序号
		1,		// ID_max,最大条纹数目 
		1,		// ID_min，最小条纹数目
		-470,	// LED灯具的真实位置,x坐标
		940,	// LED灯具的真实位置,y坐标
	};

	struct position P3 = {	// LED 序号
		7,		// ID_max,最大条纹数目 
		6,		// ID_min，最小条纹数目
		470,	// LED灯具的真实位置,x坐标
		0,	// LED灯具的真实位置,y坐标
	};

	struct position P4 = {	// LED 序号
		10,		// ID_max,最大条纹数目 
		8,		// ID_min，最小条纹数目
		-470,	// LED灯具的真实位置,x坐标
		0,	// LED灯具的真实位置,y坐标
	};	

	struct position P5 = {	// LED 序号
		100,		// ID_max,最大条纹数目 
		11,		// ID_min，最小条纹数目
		470,	// LED灯具的真实位置,x坐标
		-940,	// LED灯具的真实位置,y坐标
	};

	struct position P6 = {	//LED 序号
		3,		// ID_max,最大条纹数目 
		2,		// ID_min，最小条纹数目
		-470,	// LED灯具的真实位置,x坐标
		-940,	// LED灯具的真实位置,y坐标
	};


	// 图像读取及判断
	cv::Mat grayImage = img;
	// Mat grayImage = cv::imread("/home/rc/Image/1.BMP",0);
	// resize(grayImage,grayImage,Size(800,600),0,0,INTER_NEAREST);
	// imshow("grayImage", grayImage);

	//将图像进行二值化
	double m_threshold;//二值化阈值
	Mat matBinary;//二值化图像
	m_threshold = getThreshVal_Otsu_8u(grayImage);//获取自动阈值


	threshold(grayImage, matBinary, m_threshold, 255, 0); // 二值化  
	// imshow("matBinary", matBinary);
	// cout<<"m_threshold="<< m_threshold << '\n';


	Mat matBinary_threshold = matBinary.clone(); 

	//先膨胀后腐蚀,闭运算
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(20, 20));//定义结构元素,size要比单灯的大，才效果好
	morphologyEx(matBinary, matBinary, MORPH_CLOSE, element);


	//去除连通区域小于500的区域
	bwareaopen(matBinary, 500);
	//imshow("matBinary-1", matBinary);//用于分割的

	for (int ii = 1;ii < 7;ii++)
	{
		int X_min, X_max, Y_min, Y_max;
		Mat img_next;
		ls_LED(matBinary, X_min, X_max, Y_min, Y_max, img_next);

		//获得LED1像素中心的位置
		double Img_local_X = (X_max + X_min) / 2;
		double Img_local_Y = (Y_max + Y_min) / 2;

		//将原图中LED1部分的区域变黑
		//获取图像的行列
		double rowB = matBinary.rows;//二值化图像的行数
		double colB = matBinary.cols;//二值化图像的列数
		Mat matBinary1 = matBinary.clone();//定义一幅图像来放去除LED1的图？？？？？？？？？？？？？？？？为什么要用1做后缀


		for (double i = 0;i < rowB;i++)
		{
			for (double j = 0;j < colB;j++)
			{
				double r = pow((i - Img_local_Y), 2) + pow((j - Img_local_X), 2) - pow(((abs(X_max - X_min)) / 2 - 2), 2);//pow(x,y)计算x的y次方
				if (r - 360 > 0)//将r扩大
				{
					//LED1圆外面像素重载为原图
					matBinary1.at<uchar>(i, j) = matBinary.at<uchar>(i, j);
				}
				else
				{
					matBinary1.at<uchar>(i, j) = 0;//将第 i 行第 j 列像素值设置为255,二值化后为0和255
				}
			}
		}
		matBinary = matBinary1.clone();
		bwareaopen(matBinary, 500);//去除连通区域小于500的区域,这是必须的，因为上面的圆很有可能清不掉

		unkonwn.img_next = img_next.clone();
		unkonwn.Img_local_X = Img_local_X;
		unkonwn.Img_local_Y = Img_local_Y;
		unkonwn.matBinary = matBinary1.clone(); 
		//框框
		unkonwn.X_min = X_min;
		unkonwn.X_max = X_max;
		unkonwn.Y_min = Y_min;
		unkonwn.Y_max = Y_max;

		//imshow("matBinary_threshold", matBinary_threshold);//对二值化的图进行的复制
		unkonwn.image_cut = matBinary_threshold(Rect(unkonwn.X_min, unkonwn.Y_min, unkonwn.X_max - unkonwn.X_min, unkonwn.Y_max - unkonwn.Y_min));
		//做图像细化(有用，效果好)
		chao_thinimage(unkonwn.image_cut);
		//用findContours检测轮廓，函数将白色区域当作前景物体。所以找轮廓找到的是白色区域的轮廓
		findContours(unkonwn.image_cut, unkonwn.contours, unkonwn.hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
		unkonwn.ID = unkonwn.contours.size();

		// 根据ID判断对应的LED，并写入坐标值
		if (unkonwn.ID <= P1.max && unkonwn.ID >= P1.min)
			{unkonwn.X = P1.X;
			unkonwn.Y = P1.Y;
			unkonwn.num = 1;}
		else if (unkonwn.ID <= P2.max && unkonwn.ID >= P2.min)
			{unkonwn.X = P2.X;
			unkonwn.Y = P2.Y;
			unkonwn.num = 2;}
		else if (unkonwn.ID <= P3.max && unkonwn.ID >= P3.min)
			{unkonwn.X = P3.X;
			unkonwn.Y = P3.Y;
			unkonwn.num = 3;}
		else if (unkonwn.ID <= P4.max && unkonwn.ID >= P4.min)
			{unkonwn.X = P4.X;
			unkonwn.Y = P4.Y;
			unkonwn.num = 4;}
		else if (unkonwn.ID <= P5.max && unkonwn.ID >= P5.min)
			{unkonwn.X = P5.X;
			unkonwn.Y = P5.Y;
			unkonwn.num = 5;}
		else if (unkonwn.ID <= P6.max && unkonwn.ID >= P6.min)
			{unkonwn.X = P6.X;
			unkonwn.Y = P6.Y;
			unkonwn.num = 6;}

		// 将以上的unknown结构体的值一起赋予某个灯具，释放出unknown
		switch (ii)
		{
		case 1:
			A = unkonwn;
			break;
		case 2:
			B = unkonwn;
			break;
		case 3:
			C = unkonwn;
			break;
		case 4:
			D = unkonwn;
			break;
		case 5:
			E = unkonwn;
			break;
		case 6:
			F = unkonwn;
			break;
		}
	}

	cout << "a="<< A.ID << '\n';
	cout << "b="<< B.ID << '\n';
	cout << "c="<< C.ID << '\n';
	cout << "d="<< D.ID << '\n';
	cout << "e="<< E.ID << '\n';
	cout << "f="<< F.ID << '\n';
	// cout << "a=" << A.ID << '\n' << A.Img_local_X << '\n' << A.Img_local_Y << '\n';
	// cout << "b=" << B.ID << '\n' << B.Img_local_X << '\n' << B.Img_local_Y << '\n';
	// cout << "c=" << C.ID << '\n' << C.Img_local_X << '\n' << C.Img_local_Y << '\n';
	// cout << "d=" << D.ID << D.Img_local_X << D.Img_local_Y << '\n';
	// cout << "e=" << E.ID << E.Img_local_X << E.Img_local_Y << '\n';
	// cout << "f=" << F.ID << F.Img_local_X << F.Img_local_Y << '\n';


	// 计算位置坐标
	// 焦距
	double f = 1.5;
	// 透镜焦点在image sensor上的位置(与图像的像素有关，此数据适用于800x600)
	double Center_X = 394;
	double Center_Y = 328.5;
	// double Center_X = 395;
	// double Center_Y = 326;
	// double Center_X = 400;
	// double Center_Y = 300;
	// double Center_X = 391.8;
	// double Center_Y = 328.7;

	struct XYZ pose;
	if (C.ID > 0)
	{
		pose = three_LED(f, Center_X, Center_Y, A, B, C); 
	}
	else
	{
		pose = double_LED(f, Center_X, Center_Y, A, B);
	}
	

	
	pose.img_point = img_point;
    // cv::flip(pose.img_point,pose.img_point,0);

    //-- 第一步:检测 Oriented FAST 角点位置
    //detector->detect ( img_1,keypoints_1 );
    //circle(img_1,(100,63),55,(255,0,0),-1);
	double xxx=5*pose.x;
	double yyy=5*pose.y;
    circle(pose.img_point, Point(270+xxx, 512-yyy), 10, Scalar(0, 0, 255));
	// circle(pose.img_point, Point(200+200, 350-200), 10, Scalar(0, 0, 255));
    
    //-- 第二步:根据角点位置计算 BRIEF 描述子
    //descriptor->compute ( img_1, keypoints_1, descriptors_1 );
        
    //Mat outimg1;
    //drawKeypoints( img_1, keypoints_1, outimg1, (255,0,0), DrawMatchesFlags::DEFAULT );
    // namedWindow("picture");    
    // cv::imshow("picture",img_1);

	// cout << pose.x << '\n' << pose.y << '\n' << pose.y << '\n'<< endl;
	// 等待用户按0键退出程序  
	waitKey(0);
	return pose;

}


//////////////////////////////////////////OpenCV话题订阅//////////////////////////////////////////


//static const std::string OUTPUT = "Output"; //定义输出窗口名称 


//-----------------------------------【函数声明部分】-------------------------------------- 

//-----------------------------------【IMAGE_LISTENER_and_LOCATOR对象】------------------------------------
//            描述：接受话题“webcam/image_raw” 的图像
//            来源: http://blog.csdn.net/robogreen/article/details/50488215
//----------------------------------------------------------------------------------------------- 

//定义一个转换的类  
class IMAGE_LISTENER_and_LOCATOR  
{  
private:  
    ros::NodeHandle nh_; //定义ROS句柄  
    image_transport::ImageTransport it_; //定义一个image_transport实例  
    image_transport::Subscriber image_sub_; //定义ROS图象接收器  
	image_transport::Publisher image_pub_; 
    struct XYZ pose_value;
  
public:  
    IMAGE_LISTENER_and_LOCATOR()  
      :it_(nh_) //构造函数  
    {  
        image_sub_ = it_.subscribe("/camera/image", 1, &IMAGE_LISTENER_and_LOCATOR::convert_callback, this); //定义图象接受器，订阅话题是“camera/image”   
        image_pub_ = it_.advertise("/camera/image_show", 1); //定义ROS图象发布器
		// 初始化输入输出窗口  
		// cv::namedWindow(INPUT);  
		// cv::namedWindow(OUTPUT);  
    }  
    ~IMAGE_LISTENER_and_LOCATOR() //析构函数  
    {  
		// cv::destroyWindow(INPUT);  
		// cv::destroyWindow(OUTPUT);  
    }  
	
	//----------------------------【ROS和OpenCV的格式转换回调函数】--------------------------------------
    //      描述：这是一个ROS和OpenCV的格式转换回调函数，将图象格式从sensor_msgs/Image  --->  cv::Mat 
    //-----------------------------------------------------------------------------------------------
    void convert_callback(const sensor_msgs::ImageConstPtr& msg)   
    {  
        cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例  
  
        try  
        {  
            cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针  
        }  
        catch(cv_bridge::Exception& e)  //异常处理  
        {  
            ROS_ERROR("cv_bridge exception: %s", e.what());  
            return;  
        }  
  
        image_process(cv_ptr->image); //得到了cv::Mat类型的图象，在CvImage指针的image中，将结果传送给处理函数     
    }  
	
    //----------------------------------【图象处理主函数】----------------------------------------------
    //      描述：这是图象处理主函数，一般会把图像处理的主要程序写在这个函数中。 
    //-----------------------------------------------------------------------------------------------
    void image_process(cv::Mat img)   
    { 
       ros::Publisher chatter_pub = nh_.advertise<std_msgs::String>("location", 1000); 
       cv::Mat img_out;    
	   ros::Rate loop_rate(60); //帧率

	  /**
	   * 以下为信息输出部分，暂时采用字符串格式输出。
	   * A count of how many messages we have sent. This is used to create
	   * a unique string for each message.
	   */
	  int count = 0;
	  while (ros::ok())
	  {
		/**
		 * This is a message object. You stuff it with data, and then publish it.
		 */
		std_msgs::String msg;

		std::stringstream ss;
		
		cv::cvtColor(img, img_out, CV_RGB2GRAY);  //转换成灰度图象    
		// cv::imshow(OUTPUT, img_out);

		pose_value=Get_coordinate(img_out);

       	ss  << '\n'<< pose_value.x  << '\n'<<pose_value.y << '\n'<<pose_value.z << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());


        sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pose_value.img_point).toImageMsg();
        image_pub_.publish(msg_image);

		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */
		chatter_pub.publish(msg);
		ros::spin();

		loop_rate.sleep();
		++count;
	  	}

    }  
}; 
  


//---------------------------------------【main()函数】------------------------------------------
//      描述：主函数，发布定位信息
//      rosrun vlc_locator publisher
//----------------------------------------------------------------------------------------------- 

  
  
//主函数  
int main(int argc, char** argv)  
{  
	img_point = cv::imread ( "/home/rc/catkin_ws/src/VLC/vlc_locator/坐标纸.jpg", CV_LOAD_IMAGE_COLOR );
    ros::init(argc, argv, "IMAGE_LISTENER_and_LOCATOR");  
    IMAGE_LISTENER_and_LOCATOR obj;  
    ros::spin();
} 
  

