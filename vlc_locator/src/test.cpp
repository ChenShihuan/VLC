#include <stdio.h>
#include <stdlib.h>
#include "vlcCommonInclude.hpp"
#include "std_msgs/String.h"
#include <sstream>
#include "imageProcess.hpp"
#include "positioningCalculation.hpp"
using namespace cv;

// ID识别函数
int main() {
    cv::Mat imageLED = imread("/home/chen/图片/1000-1-1.bmp");
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

    // cv::Mat col =  (cv::Mat_<uchar>(1, 18) << 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0); 
    // resize(col,imageLED,Size(18,30),0,0,INTER_NEAREST);
    // imageLED = imageLED.t();
    // cv::cvtColor(imageLED,imageLED,cv::COLOR_BGR2GRAY);
    // imshow("input", imageLED);
    cv::Mat msgHeaderStampTest = (cv::Mat_<uchar>(1, 5) <<  0, 1, 0, 1, 0);

    // cv::Mat col = imageLED.col(imageLED.size().height / 2);
    // col = col.t();  // 转置为行矩阵
    // cv::Mat bit = convertPxielColToBit(col);
    // std::cout << "Bit = "<< bit <<std::endl;

    cv::Mat msgDate = getMsgDate(imageLED, msgHeaderStampTest);
    // cv::Mat msgDate = getMsgDate(imageLED);
    std::cout << "msgDate = "<< msgDate <<std::endl;
    cvWaitKey(0);

    return 0;
}


//     // https://stackoverflow.com/questions/32737420/multiple-results-in-opencvsharp3-matchtemplate
