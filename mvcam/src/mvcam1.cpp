#include <ros/ros.h>
#include <image_transport/image_transport.h>  
#include <sstream>  
#include <iostream>  
#include "opencv2/opencv.hpp"  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  
#include <mvsdk/CameraApi.h> // 相机SDK的API头文件，相关API的说明在该文件中可以找到

using namespace std;  
using namespace cv;  
  
int main(int argc, char **argv)  
{  
  ros::init(argc,argv,"video_publisher");  
  ros::NodeHandle nh;  
  image_transport::ImageTransport it(nh);//发布图片需要用到image_transport  
  image_transport::Publisher pub = it.advertise("camera/image", 1);  
  
  if(argc)  
  cout<<argv[1]<<endl;  
  ros::Rate loop_rate(30);  
  string path = "/home/chen/catkin_ws/Sample1.avi";  
  
  VideoCapture cap(path);//open video from the path  
  if(!cap.isOpened())  
  {  
  std::cout<<"open video failed!"<<std::endl;  
  return -1;  
  }  
  else  
  std::cout<<"open video success!"<<std::endl;  
  
  Mat frame;//this is an image  
  bool isSuccess = true;  
  while(nh.ok())  
  {  
  isSuccess = cap.read(frame);  
  if(!isSuccess)//if the video ends, then break  
  {  
  std::cout<<"video ends"<<std::endl;  
  break;  
  }  
//将opencv的图片转换成ros的sensor_msgs，然后才能发布。  
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  
  
  pub.publish(msg);  
  ros::spinOnce();  
  loop_rate.sleep();  
  }  
  return 0;  
}  

    tSdkImageResolution sRoiResolution;
    memset(&sRoiResolution,0,sizeof(sRoiResolution));
    sRoiResolution.iIndex = 0xff; // 设置成0xff表示自定义分辨率，设置成0到N表示选择预设分辨率
    sRoiResolution.iWidth = 1024; // 1024 X 768
    sRoiResolution.iHeight = 768; 
    sRoiResolution.uSkipMode = 1;
    // sRoiResolution.iWidth = 512; // 512 X 386
    // sRoiResolution.iHeight = 386; 
    // sRoiResolution.uSkipMode = 4;
    sRoiResolution.iWidthFOV = 2048;
    sRoiResolution.iHeightFOV = 1536;
    sRoiResolution.iWidthZoomHd = 0;
    sRoiResolution.iHeightZoomHd = 0;
    sRoiResolution.iHOffsetFOV = 0;
    sRoiResolution.iVOffsetFOV = 0;
    sRoiResolution.iWidthZoomSw = 0;
    sRoiResolution.iHeightZoomSw = 0;
    sRoiResolution.uBinAverageMode = 0;
    sRoiResolution.uBinSumMode = 0;
    sRoiResolution.uResampleMask = 0;
    CameraSetImageResolution(hCamera,&sRoiResolution);
    

tSdkImageResolution sRoiResolution;
memset(&sRoiResolution,0,sizeof(sRoiResolution));
sRoiResolution.iIndex = 0xff;\\设置成0xff表示自定义分辨率，设置成0到N表示选择预设分辨率
sRoiResolution.iWidth = 2048;
sRoiResolution.iWidthFOV = 2048;
sRoiResolution.iHeight = 1536;
sRoiResolution.iHeightFOV = 1536;
sRoiResolution.iWidthZoomHd = 0;
sRoiResolution.iHeightZoomHd = 0;
sRoiResolution.iHOffsetFOV = 0;
sRoiResolution.iVOffsetFOV = 0;
sRoiResolution.iWidthZoomSw = 0;
sRoiResolution.iHeightZoomSw = 0;
sRoiResolution.uBinAverageMode = 0;
sRoiResolution.uBinSumMode = 0;
sRoiResolution.uResampleMask = 0;
sRoiResolution.uSkipMode = 0;
CameraSetImageResolution(m_hCamera,&sRoiResolution);

tSdkImageResolution sRoiResolution;
memset(&sRoiResolution,0,sizeof(sRoiResolution));
sRoiResolution.iIndex = 0xff;\\设置成0xff表示自定义分辨率，设置成0到N表示选择预设分辨率
sRoiResolution.iWidth = 2048;
sRoiResolution.iWidthFOV = 2048;
sRoiResolution.iHeight = 1536;
sRoiResolution.iHeightFOV = 1536;
sRoiResolution.iWidthZoomHd = 0;
sRoiResolution.iHeightZoomHd = 0;
sRoiResolution.iHOffsetFOV = 0;
sRoiResolution.iVOffsetFOV = 0;
sRoiResolution.iWidthZoomSw = 0;
sRoiResolution.iHeightZoomSw = 0;
sRoiResolution.uBinAverageMode = 0;
sRoiResolution.uBinSumMode = 0;
sRoiResolution.uResampleMask = 0;
sRoiResolution.uSkipMode = 0;
CameraSetImageResolution(m_hCamera,&sRoiResolution);


//#include <ros/ros.h>  
//#include <image_transport/image_transport.h>  
//#include <opencv2/highgui/highgui.hpp>  
//#include <cv_bridge/cv_bridge.h>  
//#include <sstream> // for converting the command line parameter to integer  
//  
//int main(int argc, char** argv)  
//{  
//  // Check if video source has been passed as a parameter  
//  if(argv[1] == NULL)   
//    {  
//            ROS_INFO("argv[1]=NULL\n");  
//        return 1;  
//    }  
//  
//  ros::init(argc, argv, "image_publisher");  
//  ros::NodeHandle nh;  
//  image_transport::ImageTransport it(nh);  
//  image_transport::Publisher pub = it.advertise("camera/image", 1);  
//  
//  // Convert the passed as command line parameter index for the video device to an integer  
//  std::istringstream video_sourceCmd(argv[1]);  
//  int video_source;  
//  // Check if it is indeed a number  
//  if(!(video_sourceCmd >> video_source))   
//  {  
//      ROS_INFO("video_sourceCmd is %d\n",video_source);  
//      return 1;  
//  }  
//  
//  cv::VideoCapture cap(0);  
//  // Check if video device can be opened with the given index  
//  if(!cap.isOpened())   
//  {  
//      ROS_INFO("can not opencv video device\n");  
//      return 1;  
//  }  
//  cv::Mat frame;  
//  sensor_msgs::ImagePtr msg;  
//  
//  ros::Rate loop_rate(5);  
//  while (nh.ok()) {  
//    cap >> frame;  
//    // Check if grabbed frame is actually full with some content  
//    if(!frame.empty()) {  
//      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  
//      pub.publish(msg);  
//      //cv::Wait(1);  
//    }  
//  
//    ros::spinOnce();  
//    loop_rate.sleep();  
//  }  
//} 

//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>

//#include <stdio.h>

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "image_publisher");
//  ros::NodeHandle nh;
//  image_transport::ImageTransport it(nh);
//  image_transport::Publisher pub = it.advertise("camera/image", 1);

//  
//  ros::Rate loop_rate(5);
//  while (nh.ok()) {
// 	cv::Mat image = cv::imread("/home/chen/catkin_ws/src/mvsdk/Microsoft Surface Studio .jpg", CV_LOAD_IMAGE_COLOR);
//	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

//    pub.publish(msg);
//    ros::spinOnce();
//    loop_rate.sleep();
//  }
//}
