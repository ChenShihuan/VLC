/*
// Copyright 2019 
// R&C Group
*/

#ifndef positioningCalculation_hpp_
#define positioningCalculation_hpp_
// -----------------------------------【头文件包含部分】---------------------------------------  
//     描述：包含程序所依赖的头文件
// ----------------------------------------------------------------------------------------------  
#include "vlcCommonInclude.hpp"
    

// -----------------------------------------------------------------------------------------------
// **********************************************************************************************
// 
//     *********************             【定位计算函数声明部分】              *******************
// 
// **********************************************************************************************
// -----------------------------------------------------------------------------------------------

cv::Mat pointOnMap(cv::Mat imgPoint,geometry_msgs::Point point);
geometry_msgs::Point double_LED(double f,
                               double Center_X,
                               double Center_Y,
                               double Hight_of_LED,
                               double Pixel_Size,
                               struct LED D1,
                               struct LED D2);
geometry_msgs::Point three_LED(double f,
                               double Center_X,
                               double Center_Y,
                               double Hight_of_LED,
                               double Pixel_Size,
                               struct LED D1,
                               struct LED D2,
                               struct LED D3);

namespace vlp {
    class Point {
    private:
        geometry_msgs::Point point;
        std::string poseFlag;  //!< if you want to get pose ("pose", etc.)
        
    public:
        struct poseOfDoubleLED {
            geometry_msgs::Point point;
            double alpha;
        };

        Point ();
        ~Point ();
        geometry_msgs::Point VLPbyLED(const double & f,
                                const double & Center_X,
                                const double & Center_Y,
                                const double & Hight_of_LED,
                                const double & Pixel_Size,
                                const struct LED & D1,
                                const struct LED & D2);
        struct poseOfDoubleLED VLPbyLED(  //!< if you want to get pose ("pose", etc.)
                                const std::string& poseFlag,
                                const double & f,
                                const double & Center_X,
                                const double & Center_Y,
                                const double & Hight_of_LED,
                                const double & Pixel_Size,
                                const struct LED & D1,
                                const struct LED & D2);                        
        geometry_msgs::Point VLPbyLED(const double & f,
                                const double & Center_X,
                                const double & Center_Y,
                                const double & Hight_of_LED,
                                const double & Pixel_Size,
                                const struct LED & D1,
                                const struct LED & D2,
                                const struct LED & D3);                           
    };
}

#endif
