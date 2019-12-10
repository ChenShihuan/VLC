/*
// Copyright 2019
// R&C Group
*/

// -----------------------------------【头文件包含部分】---------------------------------------
//     描述：包含程序所依赖的头文件
// ----------------------------------------------------------------------------------------------
#include "imageProcess.hpp"
#include <algorithm>

// -----------------------------------------------------------------------------------------------
// **********************************************************************************************
//
//     *********************             【图像处理函数】              *******************
//
// **********************************************************************************************
// -----------------------------------------------------------------------------------------------


// threshold自动阈值，类似matlab中的graythresh。为了二值化
double getThreshVal_Otsu_8u(const cv::Mat& _src)
{
    cv::Size size = _src.size();
    if (_src.isContinuous())
    {
        size.width *= size.height;
        size.height = 1;
    }
    const int N = 256;
    int i, j, h[N] = { 0 };
    for (i = 0; i < size.height; i++)
    {
        const uchar* src = _src.data + _src.step*i;
        for (j = 0; j <= size.width - 4; j += 4)
        {
            int v0 = src[j], v1 = src[j + 1];
            h[v0]++; h[v1]++;
            v0 = src[j + 2]; v1 = src[j + 3];
            h[v0]++; h[v1]++;
        }
        for (; j < size.width; j++)
            h[src[j]]++;
    }

    double mu = 0, scale = 1. / (size.width*size.height);
    for (i = 0; i < N; i++)
        mu += i*h[i];

    mu *= scale;
    double mu1 = 0, q1 = 0;
    double max_sigma = 0, max_val = 0;

    for (i = 0; i < N; i++)
    {
        double p_i, q2, mu2, sigma;

        p_i = h[i] * scale;
        mu1 *= q1;
        q1 += p_i;
        q2 = 1. - q1;

        if (std::min(q1, q2) < FLT_EPSILON || std::max(q1, q2) > 1. - FLT_EPSILON)
            continue;

        mu1 = (mu1 + i*p_i) / q1;
        mu2 = (mu - q1*mu1) / q2;
        sigma = q1*q2*(mu1 - mu2)*(mu1 - mu2);
        if (sigma > max_sigma)
        {
            max_sigma = sigma;
            max_val = i;
        }
    }

    return max_val;
}



// 将图片中的LED逐个进行分割
void ls_LED(const Mat& _img, int& X_min, int& X_max, int& Y_min, int& Y_max, Mat& imgNext)
{
    Mat temp1= _img.clone();

    // 求xmin与xmax
    int row1 = temp1.rows;// 行数
    int col1 = temp1.cols;// 列
    int j = 0;// 注意是从0开始
    while (j < col1)// j的初值为1
    {
        double sum1 = 0.0;
        for (int i = 0;i < row1;i++)// 注意没有等于号
        {
            uchar* data1 = temp1.ptr<uchar>(i);// ptr<uchar>(i)[j]访问第i行第j列的像素
            sum1 = data1[j] + sum1;
        }// 将第j列的每一行加完
        if (sum1>-0.000001 && sum1< 0.000001)// double类型，不能写==0
        {
            j++;
        }
        else
        {
            break;// 跳出这个while循环
        }

    }
    X_min = j;

    while (j < col1)// j的初值为X_min
    {
        double sum1 = 0.0;
        for (int i = 0;i < row1;i++)
        {
            uchar* data1 = temp1.ptr<uchar>(i);// ptr<uchar>(i)[j]访问第i行第j列的像素
            sum1 = data1[j] + sum1;
        }// 将第j列的每一行XXXXXX加完
        if (sum1 != 0)
        {
            j++;
        }
        else
        {
            break;// 跳出这个while循环
        }
    }
    X_max = j;

    // 进行切割
    Mat imgCut = temp1(Rect(X_min, 0, X_max - X_min, row1));
    Mat temp = imgCut.clone();



    // 求ymin与ymax
    int row = temp.rows;// 行数
    int col = temp.cols;// 列
    int i = 0;
    while (i < row)// i的初值为1
    {
        double sum = 0.0;
        uchar* data = temp.ptr<uchar>(i);
        for (j = 0;j < col;j++)// 对每一行中的每一列像素进行相加，ptr<uchar>(i)[j]访问第i行第j列的像素
        {
            sum = data[j] + sum;
        }// 最终获得第i行的列和
        if (sum>-0.000001 && sum < 0.000001)
        {
            i++;
        }
        else
        {
            Y_min = i;
            break;// 跳出这个while循环
        }
    }
    Y_min = i;

    while (i <= row-16)// i的初值为Y_min
    {
        double sum = 0.0;
        uchar* data = temp.ptr<uchar>(i);
        for (j = 0;j < col;j++)// 对每一行中的每一列像素进行相加，ptr<uchar>(i)[j]访问第i行第j列的像素
        {
            sum = data[j] + sum;
        }// 最终获得第i行的列和
        if (sum != 0)
        {
            i++;
        }
        else
        {
            double sum6 = 0.0;
            int iiii = i + 16;
            uchar* data = temp.ptr<uchar>(iiii);
            for (j = 0;j < col;j++)// 对每一行中的每一列像素进行相加，ptr<uchar>(i)[j]访问第i行第j列的像素
            {
                sum6 = data[j] + sum6;
            }// 最终获得第i行之后20行，即iiii的列和
            if (sum6 > -0.000001 && sum6 < 0.000001)// 如果仍然为0，才跳出
            {
                Y_max = i;
                goto logo;// 跳出这个while循环
            }
            else// 否则继续执行
            {
                i++;
            }
        }
    }
    logo:
    Y_max = i;

    // 进行切割
    Mat imgCut1 = temp(Rect(0, Y_min, col, Y_max - Y_min));
    imgNext = imgCut1.clone();   // clone函数创建新的图片
}



// 起到MATLAB中，bwareaopen的功能，去除连通区域少于n的部分
void bwareaopen(Mat &data, int n)
{
    Mat labels, stats, centroids;
    connectedComponentsWithStats(data, labels, stats, centroids, 8, CV_16U);
    int regions_count = stats.rows - 1;
    int regions_size, regions_x1, regions_y1, regions_x2, regions_y2;

    for (int i = 1;i <= regions_count;i++)
    {
        regions_size = stats.ptr<int>(i)[4];
        if (regions_size < n)
        {
            regions_x1 = stats.ptr<int>(i)[0];
            regions_y1 = stats.ptr<int>(i)[1];
            regions_x2 = regions_x1 + stats.ptr<int>(i)[2];
            regions_y2 = regions_y1 + stats.ptr<int>(i)[3];

            for (int j = regions_y1;j<regions_y2;j++)
            {
                for (int k = regions_x1;k<regions_x2;k++)
                {
                    if (labels.ptr<ushort>(j)[k] == i)
                        data.ptr<uchar>(j)[k] = 0;
                }
            }
        }
    }
}



// 实现对图像的细化
void thinImage(Mat &srcimage)// 单通道、二值化后的图像
{
    using namespace std;

    vector<Point> deletelist1;
    int Zhangmude[9];
    int nl = srcimage.rows;
    int nc = srcimage.cols;
    while (true)
    {
        for (int j = 1; j < (nl - 1); j++)
        {
            uchar* data_last = srcimage.ptr<uchar>(j - 1);
            uchar* data = srcimage.ptr<uchar>(j);
            uchar* data_next = srcimage.ptr<uchar>(j + 1);
            for (int i = 1; i < (nc - 1); i++)
            {
                if (data[i] == 255)
                {
                    Zhangmude[0] = 1;
                    if (data_last[i] == 255) Zhangmude[1] = 1;
                    else  Zhangmude[1] = 0;
                    if (data_last[i + 1] == 255) Zhangmude[2] = 1;
                    else  Zhangmude[2] = 0;
                    if (data[i + 1] == 255) Zhangmude[3] = 1;
                    else  Zhangmude[3] = 0;
                    if (data_next[i + 1] == 255) Zhangmude[4] = 1;
                    else  Zhangmude[4] = 0;
                    if (data_next[i] == 255) Zhangmude[5] = 1;
                    else  Zhangmude[5] = 0;
                    if (data_next[i - 1] == 255) Zhangmude[6] = 1;
                    else  Zhangmude[6] = 0;
                    if (data[i - 1] == 255) Zhangmude[7] = 1;
                    else  Zhangmude[7] = 0;
                    if (data_last[i - 1] == 255) Zhangmude[8] = 1;
                    else  Zhangmude[8] = 0;
                    int whitepointtotal = 0;
                    for (int k = 1; k < 9; k++)
                    {
                        whitepointtotal = whitepointtotal + Zhangmude[k];
                    }
                    if ((whitepointtotal >= 2) && (whitepointtotal <= 6))
                    {
                        int ap = 0;
                        if ((Zhangmude[1] == 0) && (Zhangmude[2] == 1)) ap++;
                        if ((Zhangmude[2] == 0) && (Zhangmude[3] == 1)) ap++;
                        if ((Zhangmude[3] == 0) && (Zhangmude[4] == 1)) ap++;
                        if ((Zhangmude[4] == 0) && (Zhangmude[5] == 1)) ap++;
                        if ((Zhangmude[5] == 0) && (Zhangmude[6] == 1)) ap++;
                        if ((Zhangmude[6] == 0) && (Zhangmude[7] == 1)) ap++;
                        if ((Zhangmude[7] == 0) && (Zhangmude[8] == 1)) ap++;
                        if ((Zhangmude[8] == 0) && (Zhangmude[1] == 1)) ap++;
                        if (ap == 1)
                        {
                            if ((Zhangmude[1] * Zhangmude[7] * Zhangmude[5] == 0) && (Zhangmude[3] * Zhangmude[5] * Zhangmude[7] == 0))
                            {
                                deletelist1.push_back(Point(i, j));
                            }
                        }
                    }
                }
            }
        }
        if (deletelist1.size() == 0) break;
        for (size_t i = 0; i < deletelist1.size(); i++)
        {
            Point tem;
            tem = deletelist1[i];
            uchar* data = srcimage.ptr<uchar>(tem.y);
            data[tem.x] = 0;
        }
        deletelist1.clear();

        for (int j = 1; j < (nl - 1); j++)
        {
            uchar* data_last = srcimage.ptr<uchar>(j - 1);
            uchar* data = srcimage.ptr<uchar>(j);
            uchar* data_next = srcimage.ptr<uchar>(j + 1);
            for (int i = 1; i < (nc - 1); i++)
            {
                if (data[i] == 255)
                {
                    Zhangmude[0] = 1;
                    if (data_last[i] == 255) Zhangmude[1] = 1;
                    else  Zhangmude[1] = 0;
                    if (data_last[i + 1] == 255) Zhangmude[2] = 1;
                    else  Zhangmude[2] = 0;
                    if (data[i + 1] == 255) Zhangmude[3] = 1;
                    else  Zhangmude[3] = 0;
                    if (data_next[i + 1] == 255) Zhangmude[4] = 1;
                    else  Zhangmude[4] = 0;
                    if (data_next[i] == 255) Zhangmude[5] = 1;
                    else  Zhangmude[5] = 0;
                    if (data_next[i - 1] == 255) Zhangmude[6] = 1;
                    else  Zhangmude[6] = 0;
                    if (data[i - 1] == 255) Zhangmude[7] = 1;
                    else  Zhangmude[7] = 0;
                    if (data_last[i - 1] == 255) Zhangmude[8] = 1;
                    else  Zhangmude[8] = 0;
                    int whitepointtotal = 0;
                    for (int k = 1; k < 9; k++)
                    {
                        whitepointtotal = whitepointtotal + Zhangmude[k];
                    }
                    if ((whitepointtotal >= 2) && (whitepointtotal <= 6))
                    {
                        int ap = 0;
                        if ((Zhangmude[1] == 0) && (Zhangmude[2] == 1)) ap++;
                        if ((Zhangmude[2] == 0) && (Zhangmude[3] == 1)) ap++;
                        if ((Zhangmude[3] == 0) && (Zhangmude[4] == 1)) ap++;
                        if ((Zhangmude[4] == 0) && (Zhangmude[5] == 1)) ap++;
                        if ((Zhangmude[5] == 0) && (Zhangmude[6] == 1)) ap++;
                        if ((Zhangmude[6] == 0) && (Zhangmude[7] == 1)) ap++;
                        if ((Zhangmude[7] == 0) && (Zhangmude[8] == 1)) ap++;
                        if ((Zhangmude[8] == 0) && (Zhangmude[1] == 1)) ap++;
                        if (ap == 1)
                        {
                            if ((Zhangmude[1] * Zhangmude[3] * Zhangmude[5] == 0) && (Zhangmude[3] * Zhangmude[1] * Zhangmude[7] == 0))
                            {
                                deletelist1.push_back(Point(i, j));
                            }
                        }
                    }
                }
            }
        }
        if (deletelist1.size() == 0) break;
        for (size_t i = 0; i < deletelist1.size(); i++)
        {
            Point tem;
            tem = deletelist1[i];
            uchar* data = srcimage.ptr<uchar>(tem.y);
            data[tem.x] = 0;
        }
        deletelist1.clear();
    }
}

// 将像素列解码为数位
cv::Mat PxielToBit(const cv::Mat col) {
    // 将中间列像素计数连续相同像素，并转义
    vector<int> SamePxielCount {};
    int pxielCount = 0;  // 专门用作SamePxielCount下标
    int samePxielRange;
    int startPxiel = 0;
    int endPxiel;
    // 转义，例如001100001111转义为2244
    for (endPxiel = 0; endPxiel < col.size().width; endPxiel ++) {
        if (col.at[endPxiel] != col.at[startPxiel]) {
            samePxielRange = endPxiel - startPxiel;
            SamePxielCount.at[pxielCount] = samePxielRange;
            pxielCount++;
            startPxiel = endPxiel;
        }
    }

    // 获取转义数组中的最小值，即为一个字节所对应的像素
    int bit = std::min_element(SamePxielCount.begin, SamePxielCount.end);

    // 将转义数组再转为数据位数组
    vector<int> BitVector {};
    pxielCount = 0;
    int sameBitRaneg;

    int pxielFlag = col.at[0];  // 获取第一个像素
    //  ！！！！！！！！！！！！！此处操作存疑，像素高电平是否为1，如果不是，可能要在二值化处理，
    //  或在使用条件判断当取到高电平时赋值为1

    for (pxielCount = 0; pxielCount < SamePxielCount.size(); pxielCount++) {
        samePxielRange = SamePxielCount.at[pxielCount];
        sameBitRaneg = samePxielRange / bit;
        for (int bitCount = 0; bitCount < sameBitRaneg; bitCount ++) {
            BitVector.push_back(bitCount);
            // 在Bit末尾插入sameBitRaneg个数的像素，像素数值由pxielFlag决定
        }
        pxielFlag = 1;
        pxielFlag = ~pxielFlag;
        // 一轮填入完成后对像素标志取反，因为转义数组的相邻两个成员指代的数据位总是反的
    }
    cv::Mat Bit = Mat(BitVector, true);  // 根据文档这里貌似是一行n列
    return Bit;
}

// ID识别函数
int IDidentification(cv::Mat imageLED) {
    // 获取中间列像素，并转置为行矩阵
    // imageLED = (Mat_ < float >(3, 3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);
    cv::Mat col = imageLED.col(imageLED.size().height / 2);
    col = col.t();  // 转置为行矩阵

    // 将获取的数据位矩阵作为待匹配矩阵
    cv::Mat image_source = PxielToBit(col);

    // 用模板匹配寻找数据位中的消息头，比如101010
    cv::Mat image_template = (cv::Mat_<double>(1, 6) << 1, 0, 1, 0, 1, 0); // 根据文档PxielToBit转出来的可能是一行n列，此处行列可能为(6, 1)
    cv::Mat image_matched;
    //模板匹配
    cv::matchTemplate(image_source, image_template, image_matched, cv::TM_CCOEFF_NORMED);

    // 在两个消息头之间提取ROI区域，即位ID信息
    // minMaxLoc(image_matched, &minVal, &maxVal, &minLoc, &maxLoc, Mat());  // 用于检测矩阵中最大值和最小值的位置
    int ID;
    return ID;
}

