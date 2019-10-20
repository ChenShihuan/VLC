#ifndef positioningCalculation_hpp_
#define positioningCalculation_hpp_
//-----------------------------------【头文件包含部分】---------------------------------------  
//      描述：包含程序所依赖的头文件
//----------------------------------------------------------------------------------------------  
#include <vlcCommonInclude.hpp>
	

//-----------------------------------------------------------------------------------------------
//**********************************************************************************************
//
//      *********************             【定位计算函数声明部分】              *******************
//
//**********************************************************************************************
//-----------------------------------------------------------------------------------------------

cv::Mat pointOnMap(cv::Mat imgPoint,geometry_msgs::Point point);
geometry_msgs::Point double_LED(double f,double Center_X, double Center_Y, struct LED A,struct LED B);
geometry_msgs::Point three_LED(double f, double Center_X, double Center_Y, struct LED A,struct LED B,struct LED C);

#endif
