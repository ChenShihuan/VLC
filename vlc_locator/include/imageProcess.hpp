#ifndef imageProcess_hpp_
#define imageProcess_hpp_

#include <vlcCommonInclude.hpp>



//-----------------------------------------------------------------------------------------------
//**********************************************************************************************
//
//      *********************             【图像处理函数声明部分】              *******************
//
//**********************************************************************************************
//-----------------------------------------------------------------------------------------------
using namespace cv;

double getThreshVal_Otsu_8u(const cv::Mat& _src);
void ls_LED(const Mat& _img, int& X_min, int& X_max, int& Y_min, int& Y_max, Mat& imgNext);
void bwareaopen(Mat &data, int n);
void thinImage(Mat &srcimage);

#endif
