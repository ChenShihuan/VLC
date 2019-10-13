#ifndef imageProcess_hpp_
#define imageProcess_hpp_
 
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>  


//-----------------------------------------------------------------------------------------------
//**********************************************************************************************
//
//      *********************             【图像处理函数声明部分】              *******************
//
//**********************************************************************************************
//-----------------------------------------------------------------------------------------------
using namespace cv;

double getThreshVal_Otsu_8u(const cv::Mat& _src);
void ls_LED(const Mat& _img, int& X_min, int& X_max, int& Y_min, int& Y_max, Mat& img_next);
void bwareaopen(Mat &data, int n);
void thinImage(Mat &srcimage);

#endif
