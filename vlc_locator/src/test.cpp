#include <stdio.h>
#include <stdlib.h>
#include "vlcCommonInclude.hpp"
#include "std_msgs/String.h"
#include <sstream>
#include "imageProcess.hpp"
#include "positioningCalculation.hpp"
using namespace cv;
using namespace std;
int which_threshold=3;//一个键位来定义到底用哪种方法

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
	// cout << mat_k << endl;
	return mat_k;
}






// ID识别函数
int main() {
//**********************************先进行准确的ROI捕获******************************************
    cv::Mat imageLED1 = imread("/home/kwanwaipang/桌面/123/test2048/frame0015.jpg");
    // resize(imageLED1,imageLED1,Size(1280,960),0,0,INTER_NEAREST);
    //转换为灰度图
	Mat grayImage;//定义灰度图
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
//**********************************先进行准确的ROI捕获******************************************


    // imshow("imageLED1", imageLED1);
    //提取ROI区域
    ///////////////////////*******************通过下面来选取某个ROI区域*******************//////////////////
    // cv::Mat imageLED=imageLED1(Rect(X1_min, Y1_min, X1_max - X1_min, Y1_max - Y1_min));
    cv::Mat imageLED=imageLED1(Rect(X2_min, Y2_min, X2_max - X2_min, Y2_max - Y2_min));
    // cv::Mat imageLED=imageLED1(Rect(X3_min, Y3_min, X3_max - X3_min, Y3_max - Y3_min));

    imshow("select_ROI", imageLED);//输出对应的ROI区域
    cv::cvtColor(imageLED,imageLED,cv::COLOR_BGR2GRAY);//转换为黑白
    // imshow("灰度图", imageLED);//输出对应ROI区域的灰度图


    // std::cout << "size:" << imageLED.size() << std::endl;
    // std::cout << "row:" << imageLED.rows << std::endl;//行数目
    // std::cout << "col:" << imageLED.cols << std::endl;//列数目
    //定义一个空矩阵来存放
    cv::Mat msgDateoringal=imageLED.col(imageLED.size().height / 2);//中间列像素
    // std::cout << "中间列像素msgDate = "<< msgDateoringal.t() <<std::endl;//将消息输出出来

//**************************对于每一行的像素值求和平均**********************************############
    //method1(old)
    // for (int i=0;i!=imageLED.rows;i++)
    // {
    //     double sum1=0.0;
    //     double num=0.0;
    //     for (int j=0;j!=imageLED.cols;j++)
    //     {
    //         if(imageLED.at<uchar>(i,j)>=20)//将不为0的pixel加起来
    //         {
    //             uchar* data1 = imageLED.ptr<uchar>(i);// ptr<uchar>(i)[j]访问第i行第j列的像素
    //             sum1 = data1[j] + sum1;
    //             num++;
    //         }
    //     }
    //     msgDate.at<uchar>(i)=sum1/num;
    // }


    //method2(new)
    // 创建掩模，用于均值运算。
    int backgroundThreshold=20; //设置20为阈值
    cv::Mat maskOfimgLED;
    cv::threshold(imageLED, maskOfimgLED, backgroundThreshold, 1, cv::THRESH_BINARY);
    // 取阈值以上值的均值，逻辑是运用掩模，其中的数值为0或者1，为1的地方，计算出image中所有元素的均值，为0的地方，不计算
    cv::Mat msgDate = imageLED.col(0).t();
    int meanOfPxielRow;  //.val[0]表示第一个通道的均值
    cv::MatIterator_<uchar> it, end;
    int RowOfimgLED = 0;
    for( it = msgDate.begin<uchar>(), end = msgDate.end<uchar>(); it != end; it++) {
        meanOfPxielRow = cv::mean(imageLED.row(RowOfimgLED), maskOfimgLED.row(RowOfimgLED)).val[0];
        RowOfimgLED ++;
        // std::cout << "值 = "<< meanOfPxielRow <<std::endl;
        *it = meanOfPxielRow;
    }
    // std::cout << "插值前 = "<< msgDate <<std::endl;


    msgDate=msgDate.t();
    // cv::Mat msgDate = getMsgDate(imageLED);
    // std::cout << "取平均后选出的列像素msgDate = "<< msgDate.t() <<std::endl;//将消息输出出来
    // std::cout << "两者的差别 = "<< abs(msgDate.t()-msgDateoringal.t()) <<std::endl;//将消息输出出来
    // std::cout << "插值前信号数目 = "<< msgDate.rows <<std::endl;
//*******************************对于每一行的像素值求和平均**********************************############

    
/////////***********************采用插值的方法，对信号进行插值处理******************************************
    //关于插值，可以参考https://blog.csdn.net/guyuealian/article/details/85097633
    cv::Mat msgDate_resize;
    // std::cout << "size:" << msgDate.size() << std::endl;
    // std::cout << "row:" << msgDate.rows << std::endl;
    // std::cout << "col:" << msgDate.cols << std::endl;

    cv::resize(msgDate,msgDate_resize,Size(1,msgDate.rows*3.9),INTER_CUBIC);
    // std::cout << "插值后信号数目 = "<< msgDate_resize.rows <<std::endl;
    std::cout << "插值msgDate_resize= "<< msgDate_resize.t() <<std::endl;//将插值后的输出出来
    // std::cout << "123456= "<< msgDate_resize.size() <<std::endl;

//////////////////////////////////////////各种阈值设置的方法



//////////////////////////////////////////方法一：多项式拟合*******************************************************
    vector<Point> in_point {};//（x,y）x就是第几个像素，y就是对应像素值

    for(int i=0; i<=msgDate_resize.rows; i++)
    {
        // in_point[i].x=i;
        // in_point[i].y=msgDate_resize.at<uchar>(i);
        in_point.push_back(Point(i,msgDate_resize.at<uchar>(i)));
    }
    // std::cout << "123456= "<< in_point[324] <<std::endl;
    
    int n = 3;//**************************n次拟合*************************
	Mat mat_k = polyfit(in_point, n);

    // //计算结果可视化
	// Mat out(800, 800, CV_8UC3,Scalar::all(0));
 	// //画出原始散点
	// for (int i = 0; i < size(in_point); ++i)
	// {
	// 	Point ipt = in_point[i];
	// 	circle(out, ipt, 1, Scalar(0, 0, 255), -1);//https://www.cnblogs.com/skyfsm/p/6897313.html
	// }

	// //画出拟合曲线
	// for (int i = in_point[0].x; i < in_point[size(in_point)-1].x; ++i)
	// {
	// 	Point2d ipt;
	// 	ipt.x = i;
	// 	ipt.y = 0;
	// 	for (int j = 0; j < n + 1; ++j)
	// 	{
	// 		ipt.y += mat_k.at<double>(j, 0)*pow(i,j);
	// 	}
	// 	circle(out, ipt, 1, Scalar(255, 255, 255), CV_FILLED, CV_AA);
	// }
 
 
	// imshow("n次拟合", out);
 //////////////////////////////////////////多项式拟合*******************************************************




 /////////////////////////////////////////方法二：EVA algorithm///////////////////////////////////////////////   
    // double minVal, maxVal;//最大与最小的像素
    // int minIdx[2] = {}, maxIdx[2] = {};	//对应的坐标
    // minMaxIdx(msgDate_resize, &minVal, &maxVal, minIdx, maxIdx);//https://blog.csdn.net/qq_29796317/article/details/73136083
    // std::cout << "最大像素= "<< maxVal <<std::endl;
    // std::cout << "最大像素坐标= "<< maxIdx[0] <<std::endl;
    // std::cout << "最大像素123= "<< in_point[maxIdx[0]].y <<std::endl;

/////////////////////////////////////////方法二：EVA algorithm/////////////////////////////////////////////// 



//////////////////////////////////////////方法三：小范围的自适应阈值//////////////////////////////////////////////////
    int radius=40;
    std::vector<int> eachpixel_threshold {};
    for(int i=0;i<=msgDate_resize.rows;i++)
    {
        int pixel_Y_min;
        if (i>radius/2 && msgDate_resize.rows-i>radius/2)
        {
            pixel_Y_min=i-radius/2;
        }
        else if (i<=radius/2)
        {
            pixel_Y_min=0;
        }
        else if (msgDate_resize.rows-i<=radius/2)
        {
            pixel_Y_min=msgDate_resize.rows-radius;
        }
        
        
        cv::Mat pixel_area=msgDate_resize(Rect(0, pixel_Y_min, 1, radius));
        eachpixel_threshold.push_back(getThreshVal_Otsu_8u(pixel_area));
    }
    // std::cout << "每一个pixel的局部阈值eachpixel_threshold= "<< Mat(eachpixel_threshold, true).t() <<std::endl;//将插值后的输出出来
//////////////////////////////////////////方法三：小范围的自适应阈值///////////////////////////////////////////////





///////////////////////////////////////////////方法四:局部自适应阈值############################################
    cv::Mat binRowOfPxiel;
    cv::adaptiveThreshold(msgDate_resize, binRowOfPxiel,255, cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY, 45, 0);

    // std::cout << "binRowOfPxiel= "<< binRowOfPxiel.t()<<std::endl;
///////////////////////////////////////////////方法四:局部自适应阈值############################################



//////////////////////////////**************************采样*****************************************
    
    int sample_point=0;//采样点（0～9）

    sample_again: std::cout << "******sample_again"<<sample_point<<"   次"<<std::endl;

    std::vector<int> BitVector {};//自适应阈值
    // std::vector<int> BitVector1 {};
    std::vector<int> polysample {};//多项式阈值的采样
    std::vector<int> eachpixel_sample {};//小范围的自适应阈值采样
    std::vector<int> adaptive_threshold {};//局部自适应阈值

    double pxielFlag;
    for(int i=sample_point;i<=msgDate_resize.rows;i=i+9)
    {
        BitVector.push_back(msgDate_resize.at<uchar>(i));//数据采样
        eachpixel_sample.push_back(eachpixel_threshold[i]);//小范围的自适应阈值采样
        adaptive_threshold.push_back(binRowOfPxiel.at<uchar>(i));//局部自适应阈值采样
       
       //多项式阈值的采样
        Point2d ipt;
		ipt.x = i;
		ipt.y = 0;
		for (int j = 0; j < n + 1; ++j)
		{
			ipt.y += mat_k.at<double>(j, 0)*pow(i,j);
		}
        polysample.push_back(ipt.y);//多项式阈值的采样

    }

	// //画出采样点
	// for (int i = sample_point; i <= size(in_point); i=i+9)
	// {
	// 	Point ipt = in_point[i];
	// 	circle(out, ipt, 2, Scalar(0, 255, 0), CV_FILLED, CV_AA);//https://blog.csdn.net/yangfengman/article/details/52768862
	// }

    // //画出多项式阈值点
	// for (int i=0,b=0;i<=size(polysample);i++,b=b+9)
	// {
	// 	// Point2d ipt;
	// 	// ipt.x = i;
	// 	// ipt.y = 0;
	// 	// for (int j = 0; j < n + 1; ++j)
	// 	// {
	// 	// 	ipt.y += mat_k.at<double>(j, 0)*pow(i,j);
	// 	// }
    //     Point2d ipt;    
    //     ipt.x = in_point[b].x;
	// 	ipt.y = polysample[i];
	// 	circle(out, ipt, 2, Scalar(255, 0, 0), CV_FILLED, CV_AA);//https://blog.csdn.net/yangfengman/article/details/52768862
	// }
 
	//imshow("n次拟合", out);//opencv的坐标使得其上下是反转的。


    // Mat BitVector_Mat = Mat(BitVector, true).t();
    // std::cout << "msgDate_resize= "<< Mat(BitVector, true).t()<<std::endl;//将采样后的输出出来
    // std::cout << "m_threshold2= "<< Mat(m_threshold2, true).t()<<std::endl;//将采样后的输出出来
    // std::cout << "判决前= "<< endl <<Mat(BitVector_ploy, true).t()<<std::endl;
    // std::cout << "多项式阈值= "<< endl<<Mat(polysample, true).t()<<std::endl;

    ///////////////////////////////////方法1：多项式阈值判决polysample************************************
    std::vector<int> BitVector_ploy =BitVector;

    for (int i=0;i!=BitVector_ploy.size();i++)
    {
        if (BitVector_ploy[i]<=polysample[i])
        {
            BitVector_ploy[i]=0;
        }
        else
        {
            BitVector_ploy[i]=1;
        }
        
    }
    // std::cout << "多项式阈值判决结果= "<<endl<< Mat(BitVector_ploy, true).t()<<std::endl;
    //方法1：多项式阈值判决polysample
   ///////////////////////////////////方法1：多项式阈值判决polysample************************************



     ////////////////////////////////////方法2：小范围的自适应阈值判决*********************************
    std::vector<int> BitVector_eachpixel_ =BitVector;

    for (int i=0;i!=BitVector.size();i++)
    {
        if (BitVector_eachpixel_[i]<=eachpixel_sample[i])
        {
            BitVector_eachpixel_[i]=0;
        }
        else
        {
            BitVector_eachpixel_[i]=1;
        }
        
    }
    // std::cout << "pixel区域阈值判决= "<< Mat(BitVector_eachpixel_, true).t()<<std::endl;
    ////////////////////////////////////方法2：小范围的自适应阈值判决*********************************



    ////////////////////////////////////方法4：局部自适应阈值判决*********************************
    
    std::vector<int> BitVector_adaptive_threshold_ =adaptive_threshold;//出来的矩阵已经判决了的

    for (int i=0;i!=BitVector.size();i++)
    {
        if (BitVector_adaptive_threshold_[i]<255)
        {
            BitVector_adaptive_threshold_[i]=0;
        }
        else
        {
            BitVector_adaptive_threshold_[i]=1;
        }
        
    }
    std::cout << "局部自适应阈值判决= "<< Mat(BitVector_adaptive_threshold_, true).t()<<std::endl;
    ////////////////////////////////////方法4：局部自适应阈值判决*********************************





////////////////////////////////////////方法0：自适应阈值##########################################
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
    // std::cout << "固定阈值判决（自适应）= "<< Mat(BitVector, true).t()<<std::endl;
////////////////////////////////////////方法0：自适应阈值##########################################
    // cv::Mat msgDataVector=Mat(BitVector, true).t();



///////////////////////////////////********************************################################
//寻找header
///////////////////////////////////********************************################################
    cv::Mat msgDataVector;
    if (which_threshold==1)
    {
        msgDataVector=Mat(BitVector_ploy, true).t();//多项式阈值判断的结果
    }
    else if (which_threshold==0)
    {
        msgDataVector=Mat(BitVector, true).t();//自适应阈值判断的结果
    }
    else if (which_threshold==2)
    {
        msgDataVector=Mat(BitVector_eachpixel_, true).t();//小范围的自适应阈值判决的结果
    }
    else if (which_threshold==3)
    {
        msgDataVector=Mat(BitVector_adaptive_threshold_, true).t();//小范围的自适应阈值判决的结果
    }
    else
    {
        std::cout << "各种阈值都不行 "<< std::endl;
        return 0;
    }
    
    // change_adaptive_threshold:  std::cout << "**********************采用固定阈值判决（自适应*****************"<<std::endl;
    // change_eachpixel_threshold:    std::cout << "**********************采用pixel区域阈值判决*****************"<<std::endl;
    msgDataVector.convertTo(msgDataVector, CV_8U);
    // std::cout << "多项式自适应阈值判决= "<< msgDataVector<<std::endl;

    // // 用模板匹配寻找数据位中的消息头
    // cv::Mat Header;//字节头###########################################################################################################
    // if (which_threshold==0 || 1)
    // {
    //     Header = (cv::Mat_<uchar>(1, 5) <<  1, 0, 1, 0, 1);
    // }
    // else if (which_threshold==2 || 3)
    // {
    //     Header = (cv::Mat_<uchar>(1, 5) <<  0, 1, 0,1,0);
    // }
    cv::Mat Header = (cv::Mat_<uchar>(1, 5) <<  1, 0, 1, 0, 1);
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
        std::cout << "sample_point="<<sample_point << std::endl;
        if (which_threshold==0)//采用第一种方法
        {
            sample_point++;
            if (sample_point<=9)
            {
                goto sample_again;//重新采样
            }
            sample_point=-1;//由于下面循环先进入++，而采样范围为0～9
            which_threshold++;
        }
        if (which_threshold==1)//采用第二种方法
        {
            
            sample_point++;
            if (sample_point<=9)
            {
                goto sample_again;
            }
            sample_point=-1;
            which_threshold++;
        }
        if (which_threshold==2)
        {
            sample_point++;
            if (sample_point<=9)
            {
                goto sample_again;
            }
            sample_point=-1;
            which_threshold++;
        }
        if (which_threshold==3)
        {
            sample_point++;
            if (sample_point<=9)
            {
                goto sample_again;
            }
            sample_point=-1;
            which_threshold++;
        }
    }

    std::cout << "LED_ID="<<LED_ID << std::endl;
    // std::cout << "which_threshold="<<which_threshold << std::endl;

    switch (which_threshold)
    {
        case 0:
        std::cout << "自适应阈值判断成功" << std::endl;
        break;
        case 1:
        std::cout << "多项式判断成功" << std::endl;
        break;
        case 2:
        std::cout << "小区域自适应阈值判断成功" << std::endl;
        break;
        case 3:
        std::cout << "局部自适应阈值判断成功" << std::endl;
        break;
    }

    
    
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