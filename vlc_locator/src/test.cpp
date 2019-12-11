#include <stdio.h>
#include <stdlib.h>
#include "vlcCommonInclude.hpp"
#include "std_msgs/String.h"
#include <sstream>
#include "imageProcess.hpp"
#include "positioningCalculation.hpp"


// ID识别函数
int main() {
    cv::Mat imageLED = imread("/home/chen/图片/lena.png");
    resize(imageLED,imageLED,Size(30,30),0,0,INTER_NEAREST);
    cv::cvtColor(imageLED,imageLED,cv::COLOR_BGR2GRAY);
    double m_threshold;  // 二值化阈值
    // cv::cvtColor(matBinary,matBinary,cv::COLOR_BGR2GRAY);
    m_threshold = getThreshVal_Otsu_8u(imageLED);  // 获取自动阈值
    threshold(imageLED, imageLED, m_threshold, 255, 0);  // 二值化

    // 获取中间列像素，并转置为行矩阵
    // imageLED = (Mat_ < float >(3, 3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);
    cv::Mat col = imageLED.col(imageLED.size().height / 2);
    col = col.t();  // 转置为行矩阵

    // col =  (cv::Mat_<double>(1, 13) << 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1); 
    // col =  (cv::Mat_<float>(1, 19) << 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0); 
    cout << "col.t_gray = "<< col <<endl;
    // 将获取的数据位矩阵作为待匹配矩阵
    // 将中间列像素计数连续相同像素，并转义
    vector<int> SamePxielCount {};
    int pxielCount = 0;  // 专门用作SamePxielCount下标
    int samePxielRange;
    int startPxiel = 0;
    int endPxiel;
    int a,b;
    // 转义，例如001100001111转义为2244
    for (endPxiel = 0; endPxiel <= col.size().width; endPxiel ++) {
        // 此处存在问题：第一个像素不能读取，只能读取第二个
        a = col.at<Vec<int, 1>>(1)[startPxiel];
        b = col.at<Vec<int, 1>>(1)[endPxiel];
        if (a != b) {
            samePxielRange = endPxiel - startPxiel;
            SamePxielCount.push_back(samePxielRange);
            pxielCount++;
            startPxiel = endPxiel;
        }
    }

    // 获取转义数组中的最小值，即为一个字节所对应的像素
    int bit = *std::min_element(SamePxielCount.begin(), SamePxielCount.end());
    cout << "bit = "<< bit <<endl;

    // 将转义数组再转为数据位数组
    vector<int> BitVector {};
    pxielCount = 0;
    int sameBitRaneg;

    // 识别图像第一个像素的高低电平，转化为数据为，高电平即位1
    int pxielFlag;
    if (col.at<Vec3b>(1)[0] == 255) {
        pxielFlag = 1;
    } else {
        pxielFlag = col.at<Vec3b>(1)[0];  // 获取第一个像素
    }

    for (pxielCount = 0; pxielCount < SamePxielCount.size(); pxielCount++) {
        samePxielRange = SamePxielCount.at(pxielCount);
        sameBitRaneg = samePxielRange / bit;
        for (int bitCount = 0; bitCount < sameBitRaneg; bitCount ++) {
            BitVector.push_back(pxielFlag);
            // 在Bit末尾插入sameBitRaneg个数的像素，像素数值由pxielFlag决定
        }
        pxielFlag = !pxielFlag;
        // 一轮填入完成后对像素标志取反，因为转义数组的相邻两个成员指代的数据位总是反的
    }
    cv::Mat Bit = Mat(BitVector, true).t();
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