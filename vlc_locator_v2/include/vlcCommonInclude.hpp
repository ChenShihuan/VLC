#ifndef vlcCommonInclude_COMMON_INCLUDES_hpp_
#define vlcCommonInclude_COMMON_INCLUDES_hpp_

#include <ros/ros.h>
#include <iostream> //C++标准输入输出库  
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>  

#define pi 3.1415926
#define focalLength 1.5
#define centerXofImage 394
#define centerYofImage 328.5

using namespace cv;
using namespace std;

struct position{// LED的位置，对应不同位置的灯具
	int max;	// ID_max,最大条纹数目 	
	int min;	// ID_min，最小条纹数目
	double X;	// LED灯具的真实位置,x坐标
	double Y;	// LED灯具的真实位置,y坐标
	};

struct XYZ{	//坐标处理函数的结构体，用于放置坐标值
    double x;
	double y;
	double z;
	Mat imgPoint;
	};

struct LED{	// LED处理过程的结构体，用于存放图像处理过程中的信息以及处理结果
	int ID;								//	ID,条纹数目
	double imgLocalX, imgLocalY;	// LED在图像上的像素坐标位置，,x坐标,y坐标
	double X, Y; 						// LED灯具的真实位置,x坐标,y坐标
	Mat imgNext, matBinary;			
	int X_min, X_max, Y_min, Y_max;
	Mat imgCut;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	int num;
	};



#endif

#ifndef vlcMainInclude_
#define vlcMainInclude_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h> /*image_transport 头文件用来在ROS系统中的话题上发布和订阅图象消息 */ 
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h> /* ROS图象类型的编码函数 */ 

#endif
