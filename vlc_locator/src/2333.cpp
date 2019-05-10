#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>
 
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int main ( int argc, char** argv )
{
    
    //-- 读取图像
    
    Mat img_1 = cv::imread ( "/home/chen/catkin_ws/src/VLC/vlc_locator/74.png", CV_LOAD_IMAGE_COLOR );
    cv::flip(img_1,img_1,0);

    //-- 第一步:检测 Oriented FAST 角点位置
    //detector->detect ( img_1,keypoints_1 );
    //circle(img_1,(100,63),55,(255,0,0),-1);
    circle(img_1, Point(200+200, 350-200), 10, Scalar(0, 0, 255));
    
    //-- 第二步:根据角点位置计算 BRIEF 描述子
    //descriptor->compute ( img_1, keypoints_1, descriptors_1 );
        
    //Mat outimg1;
    //drawKeypoints( img_1, keypoints_1, outimg1, (255,0,0), DrawMatchesFlags::DEFAULT );
    namedWindow("picture");    
    cv::imshow("picture",img_1);

    cvWaitKey(0);

    return 0;
}
