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
    cv::Mat imageLED1 = imread("/home/kwanwaipang/桌面/123/test2048/frame0011.jpg");
    // imshow("input", imageLED1);
    //提取ROI区域
    cv::Mat imageLED=imageLED1(Rect(895,780,988-895,870-780));
    imshow("input_ROI", imageLED);
    // resize(imageLED,imageLED,Size(30,30),0,0,INTER_NEAREST);
    cv::cvtColor(imageLED,imageLED,cv::COLOR_BGR2GRAY);//转换为黑白
    // double m_threshold;  // 二值化阈值
    // cv::cvtColor(matBinary,matBinary,cv::COLOR_BGR2GRAY);
    //m_threshold = getThreshVal_Otsu_8u(imageLED);  // 获取自动阈值
    //cv::Mat imageLED1=imageLED.clone(); 
    //threshold(imageLED, imageLED1, m_threshold, 255, 0);  // 二值化
    //imshow("二值化", imageLED1);
    imshow("灰度图", imageLED);

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
    // cv::Mat msgHeaderStampTest = (cv::Mat_<uchar>(1, 5) <<  0, 1, 0, 1, 0);//字节头

    // cv::Mat col = imageLED.col(imageLED.size().height / 2);
    // col = col.t();  // 转置为行矩阵
    // cv::Mat bit = convertPxielColToBit(col);
    // std::cout << "Bit = "<< bit <<std::endl;

    // cv::Mat msgDate = getMsgDate(imageLED, msgHeaderStampTest);**************************8

    ///*************采用插值的方法************
    // cv::Mat col = imageLED.col(imageLED.size().height / 2);//获取中间行
    // col = col.t();  // 转置为行矩阵
    // std::cout << "size:" << imageLED.size() << std::endl;
    // std::cout << "row:" << imageLED.rows << std::endl;//行数目
    // std::cout << "col:" << imageLED.cols << std::endl;//列数目
    //定义一个空矩阵来存放
    cv::Mat msgDate=imageLED.col(imageLED.size().height / 2);
    cv::Mat msgDateoringal=msgDate.clone();
    std::cout << "原本msgDate = "<< msgDate.t() <<std::endl;//将消息输出出来
    //对于每一行的像素值求和平均
    for (int i=0;i!=imageLED.rows;i++)
    {
        double sum1=0.0;
        double num=0.0;
        for (int j=0;j!=imageLED.cols;j++)
        {
            if(imageLED.at<uchar>(i,j)>=20)//将不为0的pixel加起来
            {
                uchar* data1 = imageLED.ptr<uchar>(i);// ptr<uchar>(i)[j]访问第i行第j列的像素
                sum1 = data1[j] + sum1;
                num++;
            }
            
        }
        
        msgDate.at<uchar>(i)=sum1/num;
    }


    // cv::Mat msgDate = getMsgDate(imageLED);
    std::cout << "后面msgDate = "<< msgDate.t() <<std::endl;//将消息输出出来
    std::cout << "差别 = "<< msgDate.t()-msgDateoringal.t() <<std::endl;//将消息输出出来
    
    
    //插值
    //关于插值，可以参考https://blog.csdn.net/guyuealian/article/details/85097633
    cv::Mat msgDate_resize;
    std::cout << "size:" << msgDate.size() << std::endl;
    std::cout << "row:" << msgDate.rows << std::endl;
    std::cout << "col:" << msgDate.cols << std::endl;

    cv::resize(msgDate,msgDate_resize,Size(1,msgDate.rows*3.9),INTER_CUBIC);
    std::cout << "msgDate_resize= "<< msgDate_resize.t() <<std::endl;//将插值后的输出出来

    //采样
    std::vector<int> BitVector {};
    double pxielFlag;
    for(int i=1;i<=msgDate_resize.rows;i=i+9)
    {
        BitVector.push_back(msgDate_resize.at<uchar>(i));
    }

    Mat BitVector_Mat = Mat(BitVector, true).t();
    std::cout << "msgDate_resize= "<< Mat(BitVector, true).t()<<std::endl;//将插值后的输出出来


    //阈值判断
    double m_threshold;  // 获取自动阈值
    m_threshold = getThreshVal_Otsu_8u(BitVector_Mat);  // 获取自动阈值
    m_threshold = m_threshold + 23;
    std::cout << "m_threshold= "<< m_threshold<<std::endl;

    for (int i=0;i!=BitVector.size();i++)
    {
        if (BitVector[i]<m_threshold)
        {
            BitVector[i]=0;
        }
        else
        {
            BitVector[i]=1;
        }
        
    }
    // std::cout << "阈值判决= "<< Mat(BitVector, true).t()<<std::endl;

    cv::Mat msgDataVector=Mat(BitVector, true).t();
    msgDataVector.convertTo(msgDataVector, CV_8U);
    std::cout << "阈值判决= "<< msgDataVector<<std::endl;

    // // 用模板匹配寻找数据位中的消息头
    cv::Mat Header = (cv::Mat_<uchar>(1, 5) <<  0, 1, 0, 1, 0);//字节头
    cv::Mat result(msgDataVector.rows - Header.rows + 1, msgDataVector.cols-Header.cols + 1, CV_8U);//创建模板匹配法输出结果的矩阵
    cv::matchTemplate(msgDataVector, Header, result, CV_TM_CCOEFF_NORMED);
    //关于这个函数可以参考http://www.opencv.org.cn/opencvdoc/2.3.2/html/doc/tutorials/imgproc/histograms/template_matching/template_matching.html#id2
    cv::threshold(result, result, 0.8, 1., CV_THRESH_TOZERO);


    std::vector<int> HeaderStamp {};//存放消息头的位置

    while (true) {
        double minval, maxval, threshold = 0.8;
        cv::Point minloc, maxloc;
        cv::minMaxLoc(result, &minval, &maxval, &minloc, &maxloc);

        if (maxval >= threshold) {
            HeaderStamp.push_back(maxloc.x);
            // 漫水填充已经识别到的区域
            cv::floodFill(result, maxloc, cv::Scalar(0), 0, cv::Scalar(.1), cv::Scalar(1.));
        } else {
            break;
        }
    }


    // 在两个消息头之间提取ROI区域，即位ID信息
    int ptrHeaderStamp = 0;
    cv::Mat LED_ID;
    getROI:
    try {
        LED_ID=msgDataVector.colRange(HeaderStamp.at(ptrHeaderStamp) + Header.size().width,
                            HeaderStamp.at(ptrHeaderStamp + 1));
        //colRange（start，end），包含的范围是不保护start列，包含end列
   
    } catch ( cv::Exception& e ) {  // 异常处理
        ptrHeaderStamp++;
        // const char* err_msg = e.what();
        // std::cout << "exception caught: " << err_msg << std::endl;
        std::cout << "正常现象，切勿惊慌" << std::endl;
        goto getROI;        
    } catch ( std::out_of_range& e ) {  // 异常处理
        std::cout << "此LED图像ID无法识别" << std::endl;
    }

    std::cout << "LED_ID="<<LED_ID << std::endl;
    
    
    
    
    
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