#include <stdio.h>
#include <stdlib.h>
#include "vlcCommonInclude.hpp"
#include "std_msgs/String.h"
#include <sstream>
#include "imageProcess.hpp"
#include "positioningCalculation.hpp"
using namespace cv;
using namespace std;

struct PxielPoint {
    double i;
    int val;
};


Mat polyfit(vector<Point>& in_point, int n)
{
	int size = in_point.size();
	//所求未知数个数
	int x_num = n + 1;
	//构造矩阵U和Y
	Mat mat_u(size, x_num, CV_64F);
	Mat mat_y(size, 1, CV_64F);
 
	for (int i = 0; i < mat_u.rows; ++i)
		for (int j = 0; j < mat_u.cols; ++j)
		{
			mat_u.at<double>(i, j) = pow(in_point[i].x, j);
		}
 
	for (int i = 0; i < mat_y.rows; ++i)
	{
		mat_y.at<double>(i, 0) = in_point[i].y;
	}
 
	//矩阵运算，获得系数矩阵K
	Mat mat_k(x_num, 1, CV_64F);
	mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
	cout << mat_k << endl;
	return mat_k;
}







// ID识别函数
int main() {
    //准确的ROI捕获
    cv::Mat imageLED1 = imread("/home/kwanwaipang/桌面/123/test2048/frame0013.jpg");
    // resize(imageLED1,imageLED1,Size(1280,960),0,0,INTER_NEAREST);
    //转换为灰度图
	Mat grayImage;//�Ҷ�ͼ
	cv::cvtColor(imageLED1, grayImage, cv::COLOR_BGR2GRAY);
	//imshow("grayImage", grayImage);
    ///二值化
	double m_threshold;//
	cv::Mat matBinary;//
	m_threshold = getThreshVal_Otsu_8u(grayImage);//
	threshold(grayImage, matBinary, m_threshold, 255, 0); //
	// imshow("matBinary", matBinary);

    Mat matBinary6 = matBinary.clone();
    //
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(20, 20));
	morphologyEx(matBinary, matBinary, MORPH_CLOSE, element);

	//
	bwareaopen(matBinary, 500);
	// imshow("matBinary-1", matBinary);//连通区域

    int Img_local_X1, Img_local_Y1, Img_local_X2, Img_local_Y2, Img_local_X3, Img_local_Y3;
	Mat img1_next, matBinary11, img2_next, matBinary2, img3_next, matBinary3;
	int X1_min, X1_max, Y1_min, Y1_max, X2_min, X2_max, Y2_min, Y2_max, X3_min, X3_max, Y3_min, Y3_max;
    
    for (int ii = 1;ii < 4;ii++)
	{
		int X_min, X_max, Y_min, Y_max;
		Mat img_next;
		ls_LED(matBinary, X_min, X_max, Y_min, Y_max, img_next);

		//获得LED1像素中心的位置
		double Img_local_X = (X_max + X_min) / 2;
		double Img_local_Y = (Y_max + Y_min) / 2;

        // 将原图中LED1部分的区域变黑
        // 获取图像的行列
		double rowB = matBinary.rows;//// 二值化图像的行数
		double colB = matBinary.cols;//二值化图像的列数
		Mat matBinary1 = matBinary.clone();//����һ��ͼ������ȥ��LED1��ͼ


		for (double i = 0;i < rowB;i++)
		{
			for (double j = 0;j < colB;j++)
			{
				double r = pow((i - Img_local_Y), 2) + pow((j - Img_local_X), 2) - pow(((abs(X_max - X_min)) / 2 - 2), 2);//pow(x,y)����x��y�η�
				if (r - 360 > 0)//
				{
					//
					matBinary1.at<uchar>(i, j) = matBinary.at<uchar>(i, j);
				}
				else
				{
					matBinary1.at<uchar>(i, j) = 0;//
				}
			}
		}
		matBinary = matBinary1.clone();
		bwareaopen(matBinary, 500);//
		switch (ii)
		{
		case 1:
			img1_next = img_next.clone();
			Img_local_X1 = Img_local_X;
			Img_local_Y1 = Img_local_Y;
			matBinary11 = matBinary1.clone();
			//���
			X1_min = X_min;
			X1_max = X_max;
			Y1_min = Y_min;
			Y1_max = Y_max;
			break;
		case 2:
			img2_next = img_next.clone();
			Img_local_X2 = Img_local_X;
			Img_local_Y2 = Img_local_Y;
			matBinary2 = matBinary1.clone();
			//���
			X2_min = X_min;
			X2_max = X_max;
			Y2_min = Y_min;
			Y2_max = Y_max;
			break;
		case 3:
			img3_next = img_next.clone();
			Img_local_X3 = Img_local_X;
			Img_local_Y3 = Img_local_Y;
			matBinary3 = matBinary1.clone();
			//���
			X3_min = X_min;
			X3_max = X_max;
			Y3_min = Y_min;
			Y3_max = Y_max;
			break;
		}
	}






///////////////////////**************************************//////////////////
    // imshow("imageLED1", imageLED1);
    //提取ROI区域
    // setMouseCallback("imageLED1", on_mouse,0);
    // cv::Mat imageLED=imageLED1(Rect(895,780,988-895,870-780));
    // cv::Mat imageLED=imageLED1(Rect(X1_min, Y1_min, X1_max - X1_min, Y1_max - Y1_min));
    cv::Mat imageLED=imageLED1(Rect(X2_min, Y2_min, X2_max - X2_min, Y2_max - Y2_min));
    // cv::Mat imageLED=imageLED1(Rect(X3_min, Y3_min, X3_max - X3_min, Y3_max - Y3_min));
    // double m_threshold1 = getThreshVal_Otsu_8u(imageLED);

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

    }


    // cv::Mat msgDate = getMsgDate(imageLED);
    std::cout << "后面msgDate = "<< msgDate.t() <<std::endl;//将消息输出出来
    std::cout << "差别 = "<< msgDate.t()-msgDateoringal.t() <<std::endl;//将消息输出出来
    
    
    //插值
    //关于插值，可以参考https://blog.csdn.net/guyuealian/article/details/85097633
    cv::Mat msgDate_resize;
    // std::cout << "size:" << msgDate.size() << std::endl;
    // std::cout << "row:" << msgDate.rows << std::endl;
    // std::cout << "col:" << msgDate.cols << std::endl;

    cv::resize(msgDate,msgDate_resize,Size(1,msgDate.rows*3.9),INTER_CUBIC);
    std::cout << "msgDate_resize= "<< msgDate_resize.t() <<std::endl;//将插值后的输出出来
    // std::cout << "123456= "<< msgDate_resize.size() <<std::endl;


    vector<Point> in_point {};//（x,y）x就是第几个像素，y就是对应像素值

    for(int i=0; i<=msgDate_resize.rows; i++)
    {
        // in_point[i].x=i;
        // in_point[i].y=msgDate_resize.at<uchar>(i);
        in_point.push_back(Point(i,msgDate_resize.at<uchar>(i)));
    }
    std::cout << "123456= "<< in_point[7*9+6+5*9] <<std::endl;
    
    int n = 3;//**************************n次拟合*************************
	Mat mat_k = polyfit(in_point, n);

    //计算结果可视化
	Mat out(1500, 1500, CV_8UC3,Scalar::all(0));
 
	//画出拟合曲线
	for (int i = in_point[0].x; i < in_point[size(in_point)-1].x; ++i)
	{
		Point2d ipt;
		ipt.x = i;
		ipt.y = 0;
		for (int j = 0; j < n + 1; ++j)
		{
			ipt.y += mat_k.at<double>(j, 0)*pow(i,j);
		}
		circle(out, ipt, 1, Scalar(255, 255, 255), CV_FILLED, CV_AA);
	}
 
	//画出原始散点
	for (int i = 0; i < size(in_point); ++i)
	{
		Point ipt = in_point[i];
		circle(out, ipt, 1, Scalar(0, 0, 255), CV_FILLED, CV_AA);
	}
 
	imshow("n次拟合", out);
    

    //采样
    std::vector<int> BitVector {};
    std::vector<int> BitVector1 {};
    std::vector<int> m_threshold2 {};
    // for(int i = in_point[0].x; i < in_point[size(in_point)-1].x; ++i)
    // {
    //     Point2d ipt;
	// 	ipt.x = i;
	// 	ipt.y = 0;
	// 	for (int j = 0; j < n + 1; ++j)
	// 	{
	// 		ipt.y += mat_k.at<double>(j, 0)*pow(i,j);
	// 	}
    //     m_threshold2.push_back(ipt.y);
    // }
    // std::cout << "66666666666666666666666= "<< m_threshold2.size()<<std::endl;//将采样后的输出出来
    // std::cout << "1111111111111111111111111= "<< msgDate_resize.size()<<std::endl;

    double pxielFlag;
    for(int i=0;i<=msgDate_resize.rows;i=i+9)
    {
        BitVector.push_back(msgDate_resize.at<uchar>(i));
        BitVector1.push_back(msgDate_resize.at<uchar>(i));
        Point2d ipt;
		ipt.x = i;
		ipt.y = 0;
		for (int j = 0; j < n + 1; ++j)
		{
			ipt.y += mat_k.at<double>(j, 0)*pow(i,j);
		}
        m_threshold2.push_back(ipt.y);

    }

    Mat BitVector_Mat = Mat(BitVector, true).t();
    std::cout << "msgDate_resize= "<< Mat(BitVector, true).t()<<std::endl;//将采样后的输出出来
    std::cout << "m_threshold2= "<< Mat(m_threshold2, true).t()<<std::endl;//将采样后的输出出来


    //多项式阈值判决
    for (int i=0;i!=BitVector1.size();i++)
    {
        if (BitVector1[i]<=m_threshold2[i])
        {
            BitVector1[i]=0;
        }
        else
        {
            BitVector1[i]=1;
        }
        
    }
    std::cout << "阈值判决1= "<< Mat(BitVector1, true).t()<<std::endl;


    //阈值判断
    double m_threshold1 = getThreshVal_Otsu_8u(msgDate_resize);  // 获取自动阈值

    // double m_threshold1 = m_threshold;
    // m_threshold1=80;
    std::cout << "m_threshold1= "<< m_threshold1<<std::endl;

    for (int i=0;i!=BitVector.size();i++)
    {
        if (BitVector[i]<=m_threshold1)
        {
            BitVector[i]=0;
        }
        else
        {
            BitVector[i]=1;
        }
        
    }
    // std::cout << "阈值判决= "<< Mat(BitVector, true).t()<<std::endl;

    // cv::Mat msgDataVector=Mat(BitVector, true).t();
    cv::Mat msgDataVector=Mat(BitVector1, true).t();
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