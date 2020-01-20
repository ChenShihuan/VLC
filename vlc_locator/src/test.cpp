#include <stdio.h>
#include <stdlib.h>
#include "vlcCommonInclude.hpp"
#include "std_msgs/String.h"
#include <sstream>
#include "imageProcess.hpp"
#include "positioningCalculation.hpp"

// int main() {
//     cv::Mat imageLED = cv::imread("/media/chen/Data-SSD/8kTest2048/frame0015-2.jpg");
//     cv::cvtColor(imageLED,imageLED,cv::COLOR_BGR2GRAY);
//     imshow("input", imageLED);
//     // cv::Mat imageLED = (cv::Mat_<uchar>(13, 1) << 0, 22, 27, 73, 140, 160, 124, 76, 77, 109, 102, 62, 23);

//     imageLED = ImagePreProcessing(imageLED, 20);
//     // LEDMeanRowCrestsTroughs(imageLED);
//     LEDMeanRowThreshold(imageLED);
//     // cv::Mat bit;
//     // bit = ImagePreProcessing(imageLED, 20);
//     // std::cout << "Bit = "<< bit <<std::endl;
//     // cv::Mat msgHeaderStampTest = (cv::Mat_<uchar>(1, 5) <<  0, 1, 0, 1, 0);
//     // cv::Mat msgDate = getMsgDate(imageLED, msgHeaderStampTest);

//     // std::cout << "msgDate = "<< msgDate <<std::endl;
//     cvWaitKey(0);
//     return 0;
// }

// ID识别函数
// int main() {
//     using namespace cv;
//     cv::Mat imageLED = cv::imread("vlc_locator/8kTest/frame0015-1.jpg");
//     imshow("input", imageLED);
//     resize(imageLED,imageLED,Size(30,30),0,0,INTER_NEAREST);
//     cv::cvtColor(imageLED,imageLED,cv::COLOR_BGR2GRAY);
//     double m_threshold;  // 二值化阈值
//     cv::cvtColor(matBinary,matBinary,cv::COLOR_BGR2GRAY);

//     m_threshold = getThreshVal_Otsu_8u(imageLED);  // 获取自动阈值
//     std::cout << " m_threshold = "<<  m_threshold <<std::endl;

//     threshold(imageLED, imageLED, 100, 255, 0);  // 二值化
//     cv::imshow("二值化", imageLED);

//     // 获取中间列像素，并转置为行矩阵
//     imageLED = (Mat_ < float >(3, 3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);
//     cv::Mat col = imageLED.col(imageLED.size().height / 2);
//     col = col.t();  // 转置为行矩阵
//     col =  (cv::Mat_<double>(1, 13) << 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1); 

//     cv::Mat col =  (cv::Mat_<uchar>(1, 18) << 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0); 
//     resize(col,imageLED,Size(18,30),0,0,INTER_NEAREST);
//     imageLED = imageLED.t();
//     cv::cvtColor(imageLED,imageLED,cv::COLOR_BGR2GRAY);
//     imshow("input", imageLED);
//     cv::Mat msgHeaderStampTest = (cv::Mat_<uchar>(1, 5) <<  0, 1, 0, 1, 0);

//     cv::Mat col = imageLED.col(imageLED.size().height / 2);
//     col = col.t();  // 转置为行矩阵
//     cv::Mat bit = convertPxielColToBit(col);
//     std::cout << "Bit = "<< bit <<std::endl;

//     cv::Mat msgDate = getMsgDate(imageLED, msgHeaderStampTest);
//     cv::Mat msgDate = getMsgDate(imageLED);
//     std::cout << "msgDate = "<< msgDate <<std::endl;
//     cvWaitKey(0);

//     return 0;
// }

// -----------------------------------【Get_coordinate()函数】------------------------------------
//     描述：灰度图像传入，定位计算
// -----------------------------------------------------------------------------------------------

int main() {
    using namespace cv;
    cv::Mat img = cv::imread("vlc_locator/test image/frame0011.jpg");
    cv::cvtColor(img,img,cv::COLOR_BGR2GRAY);
    // int main() {
    // 1 2/3 4/5 6/7     9/10     11/12
    // struct LED unkonwn, A, B, C, D, E, F;
    // vector<struct LED> LEDs {A, B, C, D, E, F};
    std::vector<struct LED> LEDs {};
    geometry_msgs::Point Point;

    // 图像读取及判断
    cv::Mat grayImage = img;
    // Mat grayImage = cv::imread("/home/rc/Image/1.BMP",0);
    // resize(grayImage,grayImage,Size(800,600),0,0,INTER_NEAREST);
    // imshow("grayImage", grayImage);

    // 将图像进行二值化
    double m_threshold;  // 二值化阈值
    Mat matBinary;  // 二值化图像
    m_threshold = getThreshVal_Otsu_8u(grayImage);  // 获取自动阈值


    threshold(grayImage, matBinary, m_threshold, 255, 0);  // 二值化
    // imshow("matBinary", matBinary);
    // cout<<"m_threshold="<< m_threshold << '\n';


    Mat matBinary_threshold = matBinary.clone();

    // 先膨胀后腐蚀,闭运算
    // 定义结构元素,size要比单灯的大，才效果好
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(20, 20));
    morphologyEx(matBinary, matBinary, MORPH_CLOSE, element);


    // 去除连通区域小于500的区域
    bwareaopen(matBinary, 500);
    // imshow("matBinary-1", matBinary);// 用于分割的

    for (int ii = 1; ii < 4; ii++) {
        struct LED unkonwn;
        int X_min, X_max, Y_min, Y_max;
        Mat imgNext;
        ls_LED(matBinary, X_min, X_max, Y_min, Y_max, imgNext);

        // 获得LED1像素中心的位置
        double imgLocalX = (X_max + X_min) / 2;
        double imgLocalY = (Y_max + Y_min) / 2;


        // 将原图中LED1部分的区域变黑
        // 获取图像的行列
        double rowB = matBinary.rows;  // 二值化图像的行数
        double colB = matBinary.cols;  // 二值化图像的列数
        Mat matBinary1 = matBinary.clone();
        // 定义一幅图像来放去除LED1的图？？？？？？？？？？？？？？？？为什么要用1做后缀


        for (double i = 0; i < rowB; i++) {
            for (double j = 0; j < colB; j++) {
                double r = pow((i - imgLocalY), 2) + pow((j - imgLocalX), 2) - pow(((abs(X_max - X_min)) / 2 - 2), 2);
                // pow(x,y)计算x的y次方
                if (r - 360 > 0) {  // 将r扩大
                    // LED1圆外面像素重载为原图
                    matBinary1.at<uchar>(i, j) = matBinary.at<uchar>(i, j);
                } else {
                    matBinary1.at<uchar>(i, j) = 0;
                    // 将第 i 行第 j 列像素值设置为255,二值化后为0和255
                }
            }
        }
        matBinary = matBinary1.clone();
        bwareaopen(matBinary, 500);
        // 去除连通区域小于500的区域,这是必须的，因为上面的圆很有可能清不掉

        unkonwn.imgNext = imgNext.clone();
        unkonwn.imgLocalX = imgLocalX;
        unkonwn.imgLocalY = imgLocalY;
        unkonwn.matBinary = matBinary1.clone();
        // 框框
        unkonwn.X_min = X_min;
        unkonwn.X_max = X_max;
        unkonwn.Y_min = Y_min;
        unkonwn.Y_max = Y_max;

        // 对二值化的图进行的复制
        // imshow("matBinary_threshold", matBinary_threshold);
        unkonwn.imgCut = grayImage(Rect(unkonwn.X_min, unkonwn.Y_min, unkonwn.X_max - unkonwn.X_min, unkonwn.Y_max - unkonwn.Y_min));

        cv::Mat msgHeaderStampTest = (cv::Mat_<uchar>(1, 5) <<  1, 0, 1, 0, 1);
        cv::Mat msgDate = MsgProcess(unkonwn.imgCut, msgHeaderStampTest);

        std::cout << "msgDate = "<< msgDate <<std::endl;

        // 使用push_back将unknown结构体的值一起插入到vector末尾，赋予某个灯具，释放出unknown
        LEDs.push_back(unkonwn);
        cvWaitKey(0);
    }

    cvWaitKey(0);
    return 0;
}
