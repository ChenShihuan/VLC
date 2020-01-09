#include <stdio.h>
#include <stdlib.h>
#include "vlcCommonInclude.hpp"
#include "std_msgs/String.h"
#include <sstream>
#include "imageProcess.hpp"
#include "positioningCalculation.hpp"

int main() {
    cv::Mat imageLED = cv::imread("vlc_locator/test image/frame0010-2.jpg");
    imshow("input", imageLED);

    // imageLED = ImagePreProcessing(imageLED, 20);



    // cv::Mat bit;
    // bit = ImagePreProcessing(imageLED, 20);
    // std::cout << "Bit = "<< bit <<std::endl;
    cv::Mat msgHeaderStampTest = (cv::Mat_<uchar>(1, 5) <<  0, 1, 0, 1, 0);
    cv::Mat msgDate = getMsgDate(imageLED, msgHeaderStampTest);

    std::cout << "msgDate = "<< msgDate <<std::endl;
    cvWaitKey(0);
    return 0;
}

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


// // -----------------------------------【Get_coordinate()函数】------------------------------------
// //     描述：灰度图像传入，定位计算
// // -----------------------------------------------------------------------------------------------
// int main(int argc, char** argv) {
// // geometry_msgs::Point Get_coordinate(cv::Mat img) {
//     using namespace cv;
//     cv::Mat img = imread("vlc_locator/8kTest/8kTest/frame0000.jpg");
//     // cv::cvtColor(img,img,cv::COLOR_GRAY2BGR);
//     // cv::cvtColor(img,img,cv::COLOR_BGR2GRAY);

//     // 1 2/3 4/5 6/7     9/10     11/12
//     // struct LED unkonwn, A, B, C, D, E, F;
//     // vector<struct LED> LEDs {A, B, C, D, E, F};
//     std::vector<struct LED> LEDs {};
//     geometry_msgs::Point Point;
//     struct position P1 = {  // LED 序号
//         1,  // ID_max,最大条纹数目
//         1,  // ID_min，最小条纹数目
//         -470,  // LED灯具的真实位置,x坐标
//         940,  // LED灯具的真实位置,y坐标
//     };

//     struct position P2 = {  // LED 序号
//         10,  // ID_max,最大条纹数目
//         8,  // ID_min，最小条纹数目
//         -470,  // LED灯具的真实位置,x坐标
//         0,  // LED灯具的真实位置,y坐标
//         // -470,  // LED灯具的真实位置,x坐标
//         // 490,  // LED灯具的真实位置,y坐标
//     };

//     struct position P3 = {  // LED 序号
//         3,  // ID_max,最大条纹数目
//         2,  // ID_min，最小条纹数目
//         -470,  // LED灯具的真实位置,x坐标
//         -940,  // LED灯具的真实位置,y坐标
//         // -440,  // LED灯具的真实位置,x坐标
//         // -420,  // LED灯具的真实位置,y坐标
//     };

//     struct position P4 = {  // LED 序号
//         13,  // ID_max,最大条纹数目
//         11,  // ID_min，最小条纹数目
//         490,  // LED灯具的真实位置,x坐标
//         940,  // LED灯具的真实位置,y坐标
//     };

//     struct position P5 = {  // LED 序号
//         7,  // ID_max,最大条纹数目
//         6,  // ID_min，最小条纹数目
//         470,  // LED灯具的真实位置,x坐标
//         0,  // LED灯具的真实位置,y坐标
//         // 460,  // LED灯具的真实位置,x坐标
//         // 500,  // LED灯具的真实位置,y坐标
//     };

//     struct position P6 = {  // LED 序号
//         5,  // ID_max,最大条纹数目
//         4,  // ID_min，最小条纹数目
//         470,  // LED灯具的真实位置,x坐标
//         -940,  // LED灯具的真实位置,y坐标
//         // 470,  // LED灯具的真实位置,x坐标
//         // -420,  // LED灯具的真实位置,y坐标
//     };
//     std::vector<struct position> Position {P1, P2, P3, P4, P5, P6};

//     // 获取图片尺寸与800之比值，用于识别过于靠近边缘的灯具。
//     cv::Size s = img.size();
//     float width = s.width;
//     float ratio = width/800;

//     // 图像读取及判断
//     cv::Mat grayImage = img;
//     // Mat grayImage = cv::imread("/home/rc/Image/1.BMP",0);
//     // resize(grayImage,grayImage,Size(800,600),0,0,INTER_NEAREST);
//     // imshow("grayImage", grayImage);

//     // 将图像进行二值化
//     double m_threshold;  // 二值化阈值
//     Mat matBinary;  // 二值化图像
//     m_threshold = getThreshVal_Otsu_8u(grayImage);  // 获取自动阈值


//     threshold(grayImage, matBinary, m_threshold, 255, 0);  // 二值化
//     // imshow("matBinary", matBinary);
//     // cout<<"m_threshold="<< m_threshold << '\n';


//     Mat matBinary_threshold = matBinary.clone();

//     // 先膨胀后腐蚀,闭运算
//     // 定义结构元素,size要比单灯的大，才效果好
//     Mat element = getStructuringElement(MORPH_ELLIPSE, Size(20, 20));
//     morphologyEx(matBinary, matBinary, MORPH_CLOSE, element);


//     // 去除连通区域小于500的区域
//     bwareaopen(matBinary, 500);
//     // imshow("matBinary-1", matBinary);// 用于分割的

//     for (int ii = 1; ii < 7; ii++) {
//         struct LED unkonwn;
//         int X_min, X_max, Y_min, Y_max;
//         Mat imgNext;
//         ls_LED(matBinary, X_min, X_max, Y_min, Y_max, imgNext);

//         // 获得LED1像素中心的位置
//         double imgLocalX = (X_max + X_min) / 2;
//         double imgLocalY = (Y_max + Y_min) / 2;


//         // 将原图中LED1部分的区域变黑
//         // 获取图像的行列
//         double rowB = matBinary.rows;  // 二值化图像的行数
//         double colB = matBinary.cols;  // 二值化图像的列数
//         Mat matBinary1 = matBinary.clone();
//         // 定义一幅图像来放去除LED1的图？？？？？？？？？？？？？？？？为什么要用1做后缀


//         for (double i = 0; i < rowB; i++) {
//             for (double j = 0; j < colB; j++) {
//                 double r = pow((i - imgLocalY), 2) + pow((j - imgLocalX), 2) - pow(((abs(X_max - X_min)) / 2 - 2), 2);
//                 // pow(x,y)计算x的y次方
//                 if (r - 360 > 0) {  // 将r扩大
//                     // LED1圆外面像素重载为原图
//                     matBinary1.at<uchar>(i, j) = matBinary.at<uchar>(i, j);
//                 } else {
//                     matBinary1.at<uchar>(i, j) = 0;
//                     // 将第 i 行第 j 列像素值设置为255,二值化后为0和255
//                 }
//             }
//         }
//         matBinary = matBinary1.clone();
//         bwareaopen(matBinary, 500);
//         // 去除连通区域小于500的区域,这是必须的，因为上面的圆很有可能清不掉

//         unkonwn.imgNext = imgNext.clone();
//         unkonwn.imgLocalX = imgLocalX;
//         unkonwn.imgLocalY = imgLocalY;
//         unkonwn.matBinary = matBinary1.clone();
//         // 框框
//         unkonwn.X_min = X_min;
//         unkonwn.X_max = X_max;
//         unkonwn.Y_min = Y_min;
//         unkonwn.Y_max = Y_max;

//         // 对二值化的图进行的复制
//         // imshow("matBinary_threshold", matBinary_threshold);
//         unkonwn.imgCut = matBinary_threshold(Rect(unkonwn.X_min, unkonwn.Y_min, unkonwn.X_max - unkonwn.X_min, unkonwn.Y_max - unkonwn.Y_min));

//         // 做图像细化(有用，效果好)
//         thinImage(unkonwn.imgCut);
//         // 用findContours检测轮廓，函数将白色区域当作前景物体。所以找轮廓找到的是白色区域的轮廓
//         findContours(unkonwn.imgCut, unkonwn.contours, unkonwn.hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
//         unkonwn.ID = unkonwn.contours.size();

//         // 防止因为识别到半个灯而造成ID错误和坐标错误
//         if (X_max > 780*ratio ||
//             X_min < 20*ratio ||
//             Y_max > 580*ratio ||
//             Y_min < 20*ratio) {
//             unkonwn.ID = 0;
//             unkonwn.num = 0;
//         }

//         // 根据ID判断对应的LED，并写入坐标值
//         for (int numOfPosition = 0; numOfPosition < Position.size(); numOfPosition ++) {
//             if (unkonwn.ID <= Position.at(numOfPosition).max && unkonwn.ID >= Position.at(numOfPosition).min)
//             {unkonwn.X = Position.at(numOfPosition).X;
//             unkonwn.Y = Position.at(numOfPosition).Y;
//             unkonwn.num = numOfPosition + 1;}  // 为了遵循人类的计数方法，从第一个开始标号标记
//         }
//         // if (unkonwn.ID <= P1.max && unkonwn.ID >= P1.min)
//         //     {unkonwn.X = P1.X;
//         //     unkonwn.Y = P1.Y;
//         //     unkonwn.num = 1;}
//         // else if (unkonwn.ID <= P2.max && unkonwn.ID >= P2.min)
//         //     {unkonwn.X = P2.X;
//         //     unkonwn.Y = P2.Y;
//         //     unkonwn.num = 2;}
//         // else if (unkonwn.ID <= P3.max && unkonwn.ID >= P3.min)
//         //     {unkonwn.X = P3.X;
//         //     unkonwn.Y = P3.Y;
//         //     unkonwn.num = 3;}
//         // else if (unkonwn.ID <= P4.max && unkonwn.ID >= P4.min)
//         //     {unkonwn.X = P4.X;
//         //     unkonwn.Y = P4.Y;
//         //     unkonwn.num = 4;}
//         // else if (unkonwn.ID <= P5.max && unkonwn.ID >= P5.min)
//         //     {unkonwn.X = P5.X;
//         //     unkonwn.Y = P5.Y;
//         //     unkonwn.num = 5;}
//         // else if (unkonwn.ID <= P6.max && unkonwn.ID >= P6.min)
//         //     {unkonwn.X = P6.X;
//         //     unkonwn.Y = P6.Y;
//         //     unkonwn.num = 6;}



//         // 使用push_back将unknown结构体的值一起插入到vector末尾，赋予某个灯具，释放出unknown
//         LEDs.push_back(unkonwn);
//         // switch (ii) {
//         // case 1:
//         //     A = unkonwn;
//         //     break;
//         // case 2:
//         //     B = unkonwn;
//         //     break;
//         // case 3:
//         //     C = unkonwn;
//         //     break;
//         // case 4:
//         //     D = unkonwn;
//         //     break;
//         // case 5:
//         //     E = unkonwn;
//         //     break;
//         // case 6:
//         //     F = unkonwn;
//         //     break;
//         // }
//     }

//     std::cout << "a="<< LEDs.at(0).ID << '\n';
//     // imshow("A.imgCut",LEDs.at(0).imgCut);
//     // imshow("A.imgNext",LEDs.at(0).imgNext);
//     std::cout << "b="<< LEDs.at(1).ID << '\n';
//     std::cout << "c="<< LEDs.at(2).ID << '\n';
//     std::cout << "d="<< LEDs.at(3).ID << '\n';
//     std::cout << "e="<< LEDs.at(4).ID << '\n';
//     std::cout << "f="<< LEDs.at(5).ID << '\n';
//     // cout << "a=" << A.ID << '\n' << A.imgLocalX << '\n' << A.imgLocalY << '\n';
//     // cout << "b=" << B.ID << '\n' << B.imgLocalX << '\n' << B.imgLocalY << '\n';
//     // cout << "c=" << C.ID << '\n' << C.imgLocalX << '\n' << C.imgLocalY << '\n';
//     // cout << "d=" << D.ID << D.imgLocalX << D.imgLocalY << '\n';
//     // cout << "e=" << E.ID << E.imgLocalX << E.imgLocalY << '\n';
//     // cout << "f=" << F.ID << F.imgLocalX << F.imgLocalY << '\n';


//     // 计算位置坐标
//     // 焦距
//     double f = focalLength;
//     // 灯具高度
//     double Hight_of_LED = HightofLED;
//     double Pixel_Size = PixelSize;
//     // 透镜焦点在image sensor上的位置(与图像的像素有关，此数据适用于800x600)
//     double Center_X = centerXofImage;
//     double Center_Y = centerYofImage;

//     // 找出非0的ID，并将它在vector<struct LED> LEDs中的位置存入数组NonZeroID

//     // cout << "test ID="<< LEDs[0].ID << '\n';
//     int NonZeroID[LEDs.size()] {};
//     int getNonZeroID = 0;

//     for (int findNonZeroID = 0; findNonZeroID < LEDs.size() ; findNonZeroID++) {
//         if (LEDs.at(findNonZeroID).ID != 0) {
//             NonZeroID[getNonZeroID] = findNonZeroID;
//             std::cout << "LEDofNonZeroID="<< NonZeroID[getNonZeroID] << '\n';
//             getNonZeroID++;
//         }
//         if (getNonZeroID == 3) {
//             break;
//         }
//     }
//     // 将非0的第一个与第二个灯(以及第三个灯)代入执行定位
//     if ( NonZeroID[2] != 0 ) {
//         Point = three_LED(f, Center_X, Center_Y, Hight_of_LED, Pixel_Size,
//                             LEDs[NonZeroID[0]],
//                             LEDs[NonZeroID[1]],
//                             LEDs[NonZeroID[2]]);
//         std::cout << "3LED"<< '\n';
//     } else {
//         Point = double_LED(f, Center_X, Center_Y, Hight_of_LED, Pixel_Size,
//                             LEDs[NonZeroID[0]],
//                             LEDs[NonZeroID[1]]);
//         std::cout << "2LED"<< '\n';
//     }

//     return 0;
// }

