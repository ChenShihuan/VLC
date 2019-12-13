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

// int main() 
// {
//     // https://stackoverflow.com/questions/32737420/multiple-results-in-opencvsharp3-matchtemplate
//     cv::Mat ref;
//     // ref = cv::imread("/home/chen/图片/lena.png");
//     // cv::cvtColor(ref,ref,cv::COLOR_BGR2GRAY);
//     // ref =  (cv::Mat_<uchar>(1, 18) << 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0); // 测试用例
//     // resize(ref,ref,Size(1800,30),0,0,INTER_NEAREST);
//     // cv::imshow("reference_input", ref);
//     cv::Mat imageLED = imread("/home/chen/图片/1000-1-2.bmp");
//     imshow("input", imageLED);
//     // resize(imageLED,imageLED,Size(30,30),0,0,INTER_NEAREST);

//     double m_threshold;  // 二值化阈值
//     // cv::cvtColor(matBinary,matBinary,cv::COLOR_BGR2GRAY);
//     m_threshold = getThreshVal_Otsu_8u(imageLED);  // 获取自动阈值
//     threshold(imageLED, imageLED, m_threshold, 255, 0);  // 二值化

//     // imageLED.convertTo(imageLED, CV_8U);    
//     cv::cvtColor(imageLED,imageLED,cv::COLOR_BGR2GRAY);

    
//     cv::Mat col = imageLED.col(imageLED.size().height / 2);
//     col = col.t();  // 转置为行矩阵  
//     std::cout << "col = "<< col <<std::endl;

//     ref  = convertPxielColToBit(col);
//     ref.convertTo(ref, CV_8U);
//     std::cout << "ref = "<< ref <<std::endl;

//     cv::Mat tpl;
//     // tpl = cv::imread("/home/chen/图片/template.png");
//     // cv::cvtColor(tpl,tpl,cv::COLOR_BGR2GRAY);
//     tpl = (cv::Mat_<uchar>(1, 5) <<  0, 1, 0, 1, 0);
//     tpl.convertTo(tpl, CV_8U);
//     // resize(ref,ref,Size(300,30),0,0,INTER_NEAREST);
    
//     if (ref.empty() || tpl.empty())
//         return -1;

//     cv::Mat res(ref.rows-tpl.rows+1, ref.cols-tpl.cols+1, CV_8U);
//     cv::matchTemplate(ref, tpl, res, CV_TM_CCOEFF_NORMED);
//     cv::threshold(res, res, 0.8, 1., CV_THRESH_TOZERO);
//     std::vector<int> HeaderStamp {};

//     while (true) 
//     {
//         double minval, maxval, threshold = 0.8;
//         cv::Point minloc, maxloc;
//         cv::minMaxLoc(res, &minval, &maxval, &minloc, &maxloc);

//         if (maxval >= threshold)
//         {
//             HeaderStamp.push_back(maxloc.x);
//             // 漫水填充已经识别到的区域
//             cv::floodFill(res, maxloc, cv::Scalar(0), 0, cv::Scalar(.1), cv::Scalar(1.));
//         }
//         else
//             break;
//     }

//     cv::Mat MsgData;
//     MsgData = ref.colRange(HeaderStamp.at(0) + tpl.size().width, HeaderStamp.at(1));
//     std::cout << "MsgData = "<< MsgData <<std::endl;

//     // resize(ref,ref,Size(1800,30),0,0,INTER_NEAREST);
//     cv::imshow("reference", ref);
//     cv::waitKey();
//     return 0;
// }