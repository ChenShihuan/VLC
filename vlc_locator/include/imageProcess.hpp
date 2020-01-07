/*
// Copyright 2019 
// R&C Group
*/

#ifndef imageProcess_hpp_
#define imageProcess_hpp_
// -----------------------------------【头文件包含部分】---------------------------------------
//     描述：包含程序所依赖的头文件
// ----------------------------------------------------------------------------------------------

#include "vlcCommonInclude.hpp"

// -----------------------------------------------------------------------------------------------
// **********************************************************************************************
// 
//     *********************             【图像处理函数声明部分】              *******************
// 
// **********************************************************************************************
// -----------------------------------------------------------------------------------------------


double getThreshVal_Otsu_8u(const cv::Mat& _src);
void ls_LED(const cv::Mat& _img, int& X_min, int& X_max, int& Y_min, int& Y_max, cv::Mat& imgNext);
void bwareaopen(cv::Mat &data, int n);
void thinImage(cv::Mat &srcimage);
cv::Mat ImagePreProcessing(cv::Mat imgLED, int backgroundThreshold);
cv::Mat convertPxielRowToBit(cv::Mat col);
cv::Mat getMsgDate(const cv::Mat imageLED, cv::Mat headerStamp);

#endif
