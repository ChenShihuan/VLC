//-----------------------------------【头文件包含部分】---------------------------------------  
//      描述：包含程序所依赖的头文件
//----------------------------------------------------------------------------------------------  
#include <stdio.h>
#include <stdlib.h>
#include <vlcCommonInclude.hpp>
#include "std_msgs/String.h"
#include <sstream>
#include <imageProcess.hpp>
#include <positioningCalculation.hpp> 



//-----------------------------------【命名空间声明部分】--------------------------------------  
//      描述：包含程序所使用的命名空间  
//-----------------------------------------------------------------------------------------------  
using namespace cv;
using namespace std;

//----------------------------------·【结构体】--------------------------------------------
//      描述：定义各种结构体  
//----------------------------------------------------------------------------------------------- 
vector<struct LED> LEDs {};
struct XYZ poseValue;
Mat imgPoint;

//-----------------------------------【Get_coordinate()函数】------------------------------------
//      描述：灰度图像传入，定位计算
//-----------------------------------------------------------------------------------------------  
struct XYZ Get_coordinate(cv::Mat img)
// int main()
// 1 2/3 4/5 6/7     9/10     11/12
{
	struct LED unkonwn,A,B,C,D,E,F;
	// cout << "111" << '\n';
	struct XYZ pose;
	struct position P1 = {	// LED 序号
		7,		// ID_max,最大条纹数目 
		6,		// ID_min，最小条纹数目
		-470,	// LED灯具的真实位置,x坐标
		940,	// LED灯具的真实位置,y坐标
	};

	struct position P2 = {	// LED 序号
		9,		// ID_max,最大条纹数目 
		8,		// ID_min，最小条纹数目
		// -470,	// LED灯具的真实位置,x坐标
		// 0,	// LED灯具的真实位置,y坐标
		-470,	// LED灯具的真实位置,x坐标
		490,	// LED灯具的真实位置,y坐标
	};

	struct position P3 = {	// LED 序号
		3,		// ID_max,最大条纹数目 
		2,		// ID_min，最小条纹数目
		// -470,	// LED灯具的真实位置,x坐标
		// -940,	// LED灯具的真实位置,y坐标
		-440,	// LED灯具的真实位置,x坐标
		-420,	// LED灯具的真实位置,y坐标
	};

	struct position P4 = {	// LED 序号
		100,		// ID_max,最大条纹数目 
		11,		// ID_min，最小条纹数目
		490,	// LED灯具的真实位置,x坐标
		940,	// LED灯具的真实位置,y坐标
	};

	struct position P5 = {	// LED 序号
		1,		// ID_max,最大条纹数目 
		1,		// ID_min，最小条纹数目
		// 470,	// LED灯具的真实位置,x坐标
		// 0,	// LED灯具的真实位置,y坐标
		460,	// LED灯具的真实位置,x坐标
		500,	// LED灯具的真实位置,y坐标
	};

	struct position P6 = {	//LED 序号
		5,		// ID_max,最大条纹数目 
		4,		// ID_min，最小条纹数目
		// 470,	// LED灯具的真实位置,x坐标
		// -940,	// LED灯具的真实位置,y坐标
		470,	// LED灯具的真实位置,x坐标
		-420,	// LED灯具的真实位置,y坐标
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
		Mat imgNext;
		ls_LED(matBinary, X_min, X_max, Y_min, Y_max, imgNext);

		//获得LED1像素中心的位置
		double imgLocalX = (X_max + X_min) / 2;
		double imgLocalY = (Y_max + Y_min) / 2;


		//将原图中LED1部分的区域变黑
		//获取图像的行列
		double rowB = matBinary.rows;//二值化图像的行数
		double colB = matBinary.cols;//二值化图像的列数
		Mat matBinary1 = matBinary.clone();//定义一幅图像来放去除LED1的图？？？？？？？？？？？？？？？？为什么要用1做后缀


		for (double i = 0;i < rowB;i++)
		{
			for (double j = 0;j < colB;j++)
			{
				double r = pow((i - imgLocalY), 2) + pow((j - imgLocalX), 2) - pow(((abs(X_max - X_min)) / 2 - 2), 2);//pow(x,y)计算x的y次方
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

		unkonwn.imgNext = imgNext.clone();
		unkonwn.imgLocalX = imgLocalX;
		unkonwn.imgLocalY = imgLocalY;
		unkonwn.matBinary = matBinary1.clone(); 
		//框框
		unkonwn.X_min = X_min;
		unkonwn.X_max = X_max;
		unkonwn.Y_min = Y_min;
		unkonwn.Y_max = Y_max;

		//imshow("matBinary_threshold", matBinary_threshold);//对二值化的图进行的复制
		unkonwn.imgCut = matBinary_threshold(Rect(unkonwn.X_min, unkonwn.Y_min, unkonwn.X_max - unkonwn.X_min, unkonwn.Y_max - unkonwn.Y_min));
		//做图像细化(有用，效果好)
		thinImage(unkonwn.imgCut);
		//用findContours检测轮廓，函数将白色区域当作前景物体。所以找轮廓找到的是白色区域的轮廓
		findContours(unkonwn.imgCut, unkonwn.contours, unkonwn.hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
		unkonwn.ID = unkonwn.contours.size();

		if (X_max>780 || X_min<20 || Y_max>580 || Y_min<20){ //防止因为识别到半个灯而造成ID错误和坐标错误
			unkonwn.ID = 0;
			unkonwn.num = 0;
		}
		
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
	// cout << "a=" << A.ID << '\n' << A.imgLocalX << '\n' << A.imgLocalY << '\n';
	// cout << "b=" << B.ID << '\n' << B.imgLocalX << '\n' << B.imgLocalY << '\n';
	// cout << "c=" << C.ID << '\n' << C.imgLocalX << '\n' << C.imgLocalY << '\n';
	// cout << "d=" << D.ID << D.imgLocalX << D.imgLocalY << '\n';
	// cout << "e=" << E.ID << E.imgLocalX << E.imgLocalY << '\n';
	// cout << "f=" << F.ID << F.imgLocalX << F.imgLocalY << '\n';


	// 计算位置坐标
	// 焦距
	double f = focalLength;
	// 透镜焦点在image sensor上的位置(与图像的像素有关，此数据适用于800x600)
	double Center_X = centerXofImage;
	double Center_Y = centerYofImage;
	// double Center_X = 395;
	// double Center_Y = 326;
	// double Center_X = 400;
	// double Center_Y = 300;
	// double Center_X = 391.8;
	// double Center_Y = 328.7;

	struct LED D1,D2;

	if (C.ID == 0){
		D1=A;
		D2=B;
	}
	else
	{
		if (A.imgLocalX == C.imgLocalX){
			D1=A;
			D2=B;
		}
		else
		{
			D1=A;
			D2=C;
		}		
	}
	pose = double_LED(f, Center_X, Center_Y, D1, D2);
	
	// pose = three_LED(f, Center_X, Center_Y, A, B, C);

	
	pose.imgPoint = imgPoint;
    // cv::flip(pose.imgPoint,pose.imgPoint,0);

    //-- 第一步:检测 Oriented FAST 角点位置
    //detector->detect ( img_1,keypoints_1 );
    //circle(img_1,(100,63),55,(255,0,0),-1);
	double xxx=5*pose.x;
	double yyy=5*pose.y;
    circle(pose.imgPoint, Point(270+xxx, 512-yyy), 10, Scalar(0, 0, 255));
	// circle(pose.imgPoint, Point(200+200, 350-200), 10, Scalar(0, 0, 255));
    
	waitKey(0);
	return pose;

}


//////////////////////////////////////////OpenCV话题订阅//////////////////////////////////////////


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
	ros::Publisher msgPointPub;

    struct XYZ poseValue;
  
public:  
    IMAGE_LISTENER_and_LOCATOR()  
      :it_(nh_) //构造函数  
    {  
        image_sub_ = it_.subscribe("/mvcam/image", 1, &IMAGE_LISTENER_and_LOCATOR::convert_callback, this); //定义图象接受器，订阅话题是“camera/image”   
        image_pub_ = it_.advertise("/locator/image_show", 1); //定义ROS图象发布器
		msgPointPub = nh_.advertise<geometry_msgs::PointStamped>("location", 1000);
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
       
       cv::Mat img_out;    
	   ros::Rate loop_rate(60); //帧率

	  int count = 0;
	  while (ros::ok())
	  {
		/**
		 * This is a message object. You stuff it with data, and then publish it.
		 */
		std_msgs::String msg;
		geometry_msgs::PointStamped msgPointStamped;
		std::stringstream ss;
		
		cv::cvtColor(img, img_out, CV_RGB2GRAY);  //转换成灰度图象    
		// cv::imshow(OUTPUT, img_out);

		poseValue=Get_coordinate(img_out);

		ss  << '\n'<< poseValue.x  << '\n'<<poseValue.y << '\n'<<poseValue.z << count;
		msg.data = ss.str();
		msgPointStamped.header.stamp = ros::Time::now();
		msgPointStamped.header.frame_id = "odom";
		// 单位采用m，poseValue为cm，除以100转化为标准的m
		msgPointStamped.point.x = (poseValue.x/100);
		msgPointStamped.point.y = (poseValue.y/100);
		msgPointStamped.point.z = (poseValue.z/100);

		msgPointPub.publish(msgPointStamped);
		
		ROS_INFO("%s", msg.data.c_str());


		sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", poseValue.imgPoint).toImageMsg();
		image_pub_.publish(msg_image);

		
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
    imgPoint = cv::imread ( "/home/chen/catkin_ws/src/VLC/vlc_locator/坐标纸.jpg", CV_LOAD_IMAGE_COLOR );
	ros::init(argc, argv, "IMAGE_LISTENER_and_LOCATOR");  
    IMAGE_LISTENER_and_LOCATOR obj;  
    ros::spin();
} 
  
  

