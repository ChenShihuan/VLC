#include <stdio.h>
#include <stdlib.h>
#include "vlcCommonInclude.hpp"
#include "std_msgs/String.h"
#include <sstream>
#include "imageProcess.hpp"
#include "positioningCalculation.hpp"


// ID识别函数
int main() {
    cv::Mat imageLED = imread("/home/chen/图片/800-1.bmp");
    imshow("input", imageLED);
    // resize(imageLED,imageLED,Size(30,30),0,0,INTER_NEAREST);
    cv::cvtColor(imageLED,imageLED,cv::COLOR_BGR2GRAY);
    double m_threshold;  // 二值化阈值
    // cv::cvtColor(matBinary,matBinary,cv::COLOR_BGR2GRAY);
    m_threshold = getThreshVal_Otsu_8u(imageLED);  // 获取自动阈值
    threshold(imageLED, imageLED, m_threshold, 255, 0);  // 二值化

    // 获取中间列像素，并转置为行矩阵
    // imageLED = (Mat_ < float >(3, 3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);
    // cv::Mat col = imageLED.col(imageLED.size().height / 2);
    // col = col.t();  // 转置为行矩阵
    // col =  (cv::Mat_<double>(1, 13) << 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1); 
    // col =  (cv::Mat_<uchar>(1, 18) << 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0); 
   
    cv::Mat Bit = PxielToBit(imageLED);
    cout << "Bit = "<< Bit <<endl;

    // 用模板匹配寻找数据位中的消息头，比如101010
    // cv::Mat image_template = (cv::Mat_<double>(1, 6) << 1, 0, 1, 0, 1, 0); // 根据文档PxielToBit转出来的可能是一行n列，此处行列可能为(6, 1)
    // cout << "image_template = "<< image_template <<endl;
    // cv::Mat image_matched;
    // //模板匹配
    // cv::matchTemplate(image_source, image_template, image_matched, cv::TM_CCOEFF_NORMED);

    // 在两个消息头之间提取ROI区域，即位ID信息
    // minMaxLoc(image_matched, &minVal, &maxVal, &minLoc, &maxLoc, Mat());  // 用于检测矩阵中最大值和最小值的位置
    cvWaitKey(0);

    return 0;
}