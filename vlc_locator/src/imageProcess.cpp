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
    using namespace cv;
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
void ls_LED(const cv::Mat& _img, int& X_min, int& X_max, int& Y_min, int& Y_max, cv::Mat& imgNext)
{
    using namespace cv;
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
void bwareaopen(cv::Mat &data, int n)
{
    using namespace cv;
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
void thinImage(cv::Mat &srcimage)// 单通道、二值化后的图像
{
    using namespace cv;
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


std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

/* -------------------【 LED图像预处理 】----------------
功能：
    LED图像预处理，从原LED图像计算每行非0像素均值（理论上非0像素，实际上是某个阈值以上，目的是排除背景）
    统计为列矩阵，并进行插值3.9倍处理
输入数据类型：
    cv::Mat imgLED 切割出来的LED图像
    int threshold 背景阈值，用于蒙版处理，以提取出LED的形状
输出数据类型：
    cv::Mat meanRowOfPxiel 由每行均值组成的行矩阵，float数据类型
        因为要为了平滑的效果，需要更高精度的数据类型
------------------------------------------------------------*/
cv::Mat_<float>::Mat ImagePreProcessing(cv::Mat imgLED, int backgroundThreshold) {
    // cv::cvtColor(imgLED,imgLED,cv::COLOR_BGR2GRAY);
    // 创建掩模，用于均值运算。
    cv::Mat maskOfimgLED;
    cv::threshold(imgLED, maskOfimgLED, backgroundThreshold, 255, cv::THRESH_BINARY);

    // 用作显示，不参与函数功能
    cv::imshow("maskOfimgLED", maskOfimgLED);

    imgLED.convertTo(imgLED, CV_32F);

    // 取阈值以上值的均值，逻辑是运用掩模，其中的数值为0或者1，为1的地方，计算出image中所有元素的均值，为0的地方，不计算
    cv::Mat meanRowOfPxiel(imgLED.col(0).t().size(),CV_32FC1);
    int meanOfPxielRow;  //.val[0]表示第一个通道的均值
    cv::MatIterator_<float> it, end;
    int RowOfimgLED = 0;
    for( it = meanRowOfPxiel.begin<float>(), end = meanRowOfPxiel.end<float>(); it != end; it++) {
        meanOfPxielRow = cv::mean(imgLED.row(RowOfimgLED), maskOfimgLED.row(RowOfimgLED)).val[0];
        RowOfimgLED ++;
        std::cout << "值 = "<< meanOfPxielRow <<std::endl;
        *it = meanOfPxielRow;
    }
    std::cout << "插值前 = "<< meanRowOfPxiel <<std::endl;
    std::cout << "meanRowOfPxiel.rows = "<< meanRowOfPxiel.cols <<std::endl;

    // 插值
    cv::resize(meanRowOfPxiel, meanRowOfPxiel, cv::Size(meanRowOfPxiel.cols*4, 1), cv::INTER_CUBIC);
    std::cout << "插值 = "<< meanRowOfPxiel <<std::endl;

    // 用作显示，不参与函数功能
    cv::Mat meanShow(meanRowOfPxiel.size(),CV_32FC1);
    cv::resize(meanRowOfPxiel, meanShow, cv::Size(meanRowOfPxiel.cols, 100), cv::INTER_CUBIC);
    meanShow.convertTo(meanShow, CV_8U);
    cv::imshow("meanRowOfPxiel", meanShow);

    return meanRowOfPxiel;
 }

/* -------------------【 平移处理 】----------------
功能：
    图像平移处理，移动后暴露的部分以0填充
// 输入数据类型：
    cv::Mat frame 已由列矩阵转置为行矩阵的像素列
    int shiftCol 列的平移值，+右-左
    int shiftRow 行平移，+下-上
输出数据类型：
    cv::Mat row 已由列矩阵转置为行矩阵的数位
------------------------------------------------------------*/
cv::Mat matShift(cv::Mat frame, int shiftCol, int shiftRow) {
    using namespace cv;
    // std::cout << "平移前 = "<< frame <<std::endl;
    cv::Mat out = cv::Mat::zeros(frame.size(), frame.type());
    cv::Rect source = cv::Rect(max(0,-shiftCol),max(0,-shiftRow), frame.cols-abs(shiftCol),frame.rows-abs(shiftRow));
    cv::Rect target = cv::Rect(max(0,shiftCol),max(0,shiftRow),frame.cols-abs(shiftCol),frame.rows-abs(shiftRow));
    frame(source).copyTo(out(target));
    // std::cout << "平移后 = "<< out <<std::endl;
    return out;
}


/* -------------------【 寻找波峰波谷处理 】----------------
功能：
    寻找LED行均值的波峰波谷位置坐标
输入数据类型：
    cv::Mat imgRow 已由列矩阵转置为行矩阵的像素列，注意，必须要float类型！
        因为要为了平滑的效果，需要更高精度的数据类型
输出数据类型：
    cv::Mat NonZeroLocations 波峰波谷所在的坐标
------------------------------------------------------------*/
cv::Mat LEDMeanRowCrestsTroughs(const cv::Mat_<float>::Mat imgRow, int BlurSize) {
    // 寻找所有的极大极小值（也就是波峰和波谷）
    cv::Mat imgRowBlur;

    // 平滑，均值滤波，作用是消除曲线上小的抖动
    // cv::GaussianBlur(imgRow, imgRowBlur, cv::Size(21,1), 0, 0 );
    // 对于2048图像，BlurSize取15
    cv::blur(imgRow, imgRowBlur, cv::Size(BlurSize,1));
    // std::cout << "平滑 = "<< imgRowBlur <<std::endl;

    // 用作显示，不参与函数功能
    cv::Mat imgRowBlurShow;
    cv::resize(imgRowBlur, imgRowBlurShow, cv::Size(imgRowBlur.cols, 100), cv::INTER_CUBIC);
    imgRowBlurShow.convertTo(imgRowBlurShow, CV_8U);
    cv::imshow("imgRowBlurShow", imgRowBlurShow);

    cv::Mat imgRowRightShift = matShift(imgRowBlur, 1, 0);
    // std::cout << "右移 = "<< imgRowRightShift <<std::endl;

    cv::Mat difference;
    cv::subtract(imgRowBlur, imgRowRightShift, difference);
    // std::cout << "计算差值 = "<< difference <<std::endl;

    cv::threshold(difference, difference, 0, 255, cv::THRESH_BINARY);
    difference.convertTo(difference, CV_8U);
    // std::cout << "二值化差值 = "<< difference <<std::endl;

    cv::Mat differenceLeftShift = matShift(difference, -1, 0);
    // std::cout << "二值化差值左移 = "<< differenceLeftShift <<std::endl;

    cv::Mat CrestsTroughs;
    cv::bitwise_xor(difference, differenceLeftShift, CrestsTroughs);

    // 为末尾加上一个峰值标记，以便无需改动二值化处理的逻辑而不要漏掉最后一个区域
    CrestsTroughs = CrestsTroughs.t();
    cv::Mat endRow = (cv::Mat_<uchar>(1, 1) << 255);
    CrestsTroughs.push_back(endRow);
    CrestsTroughs = CrestsTroughs.t();
    // std::cout << "异或 = "<< CrestsTroughs <<std::endl;

    // 用作显示，不参与函数功能
    cv::Mat CrestsTroughsShow;
    cv::resize(CrestsTroughs, CrestsTroughsShow, cv::Size(CrestsTroughs.cols, 100), cv::INTER_CUBIC);
    cv::imshow("CrestsTroughsShow", CrestsTroughsShow);

    // 寻找并返回非0元素的位置，即为波峰波谷
    cv::Mat NonZeroLocations;
    cv::findNonZero(CrestsTroughs, NonZeroLocations);
    std::cout << "Non-Zero Locations = " << NonZeroLocations << std::endl;

    return NonZeroLocations;
}

/* -------------------【 二值化处理 】----------------
功能：
    二值化处理
输入数据类型：
    cv::Mat row 已由列矩阵转置为行矩阵的像素列
    cv::Mat NonZeroLocations LED行均值的波峰波谷位置坐标
    int backgroundThreshold 背景阈值
    int backgroundCompensation 背景补偿值
        为了补偿宽条纹处最小值为0所造成的二值化阈值偏低，故在此进行调节。
        此种最小值应该指定为何值，根据观察进行配置，一般的指导规则是大于去除背景
        的阈值（即ImagePreProcessing函数中的backgroundThreshold参数），
        小于或等于其它未被去除背景的区间的最小值。例如本用例中backgroundThreshold
        为20，其他未被去除背景的区间的最小值在90以上，故在此经过检验后确定取40
输出数据类型：
    cv::Mat row 已由列矩阵转置为行矩阵的数位
------------------------------------------------------------*/
cv::Mat LEDMeanRowThreshold(cv::Mat imgRow, cv::Mat NonZeroLocations, 
                            int backgroundThreshold, 
                            int backgroundCompensation) {
    cv::Mat ROI;
    cv::Rect roiRange;
    int roiThreshold, roiStart, roiEnd;
    double minVal, maxVal;
    roiStart = 0;
    for (int i = 0; i < NonZeroLocations.size().height; i++){
        // 逐个读取NonZeroLocations数据库
        cv::Point pnt = NonZeroLocations.at<cv::Point>(i);
        roiEnd = pnt.x;
        // std::cout << "roiEnd = " << roiEnd << std::endl;

        // 根据读取结果划定本次二值化的ROI区域
        roiRange = cv::Rect(roiStart, 0, (roiEnd - roiStart), 1);
        ROI = imgRow(roiRange);
        // std::cout << "ROI_1 = "<< ROI <<std::endl;

        // 获取每个区间的最大最小值
        cv::minMaxIdx(ROI, &minVal, &maxVal);

        // 为了补偿宽条纹处最小值为0所造成的二值化阈值偏低，故在此进行调节
        // 此种最小值应该指定为何值，根据观察进行配置，一般的指导规则是大于去除背景
        // 的阈值（即ImagePreProcessing函数中的backgroundThreshold参数），
        // 小于或等于其它未被去除背景的区间的最小值。例如本用例中backgroundThreshold
        // 为20，其他未被去除背景的区间的最小值在90以上，故在此经过检验后确定取40
        if (minVal < backgroundThreshold){
            minVal = backgroundCompensation;
            if (maxVal < minVal){
                maxVal = minVal;
            }
        }

        std::cout << "minVal = "<< minVal <<std::endl;
        std::cout << "maxVal = "<< maxVal <<std::endl;

        // 为了应对宽条纹中间没有被平滑掉的起伏，只有在判断区间极值大于此阈值时才会执行二值化
        if ((maxVal - minVal) > 30) {
            roiThreshold = ((minVal + maxVal) / 2);
            // std::cout << "roiThreshold = "<< roiThreshold <<std::endl;

            cv::threshold(ROI, ROI, roiThreshold, 255, cv::THRESH_BINARY);
            // std::cout << "ROI_2 = "<< ROI <<std::endl;
        }

        // 将此区域二值化结果复制回原图中
        ROI.copyTo(imgRow(roiRange));

        roiStart = roiEnd;
    }

    // 将上面循环中没有处理的部分统一处理，因为没有平滑掉的起伏只位于宽条纹中间，
    // 因此对应暗部和亮部的未处理值差别比较大，大致取中间值进行二值化即可，而已完成
    // 二值化的部分由于已经是0和255，因而不受影响
    cv::threshold(imgRow, imgRow, 40, 255, cv::THRESH_BINARY);
    imgRow.convertTo(imgRow, CV_8U);

    // 用作显示，不参与函数功能
    cv::Mat imgRowShow;
    cv::resize(imgRow, imgRowShow, cv::Size(imgRow.cols, 100), cv::INTER_CUBIC);
    cv::imshow("LEDMeanRowThreshold", imgRowShow);
    std::cout << "imgRow = "<< imgRow <<std::endl;

    return imgRow;
}

/* -------------------【 将像素列解码为数位 】----------------
功能：
    将输入的已由列矩阵转置为行矩阵的像素列解码为数位
输入数据类型：
    cv::Mat row 已由列矩阵转置为行矩阵的像素列
    int samplePoint  // 采样点起始位置
输出数据类型：
    cv::Mat row 已由列矩阵转置为行矩阵的数位，由vector通过Mat(BitVector, true).t()生成
------------------------------------------------------------*/
cv::Mat convertPxielRowToBitBySample(cv::Mat row) {
     int samplePoint;
    // 将中间列像素计数连续相同像素，并转义，例如001100001111转义为2244
    std::vector<int> SamePxielCount {};
    int pxielCount = 0;
    cv::MatIterator_<uchar> start, it, end;
    for( it = row.begin<uchar>(), end = row.end<uchar>(), start = it; it != end; it++) {
        if (*start != *it) {
            std::cout << "pxielCount = "<< pxielCount <<std::endl;
            SamePxielCount.push_back(pxielCount);
            pxielCount = 1;
            start = it;
        } else {
            pxielCount++;
        }
    }
    // 对最后一个像素特殊处理，因为不能适用前面的条件判断
    pxielCount++;
    std::cout << "pxielCount = "<< pxielCount <<std::endl;
    SamePxielCount.push_back(pxielCount);

    if (SamePxielCount.at(0) >= 5){
    samplePoint = SamePxielCount.at(0) - 5;        
    } else {
        samplePoint = SamePxielCount.at(0) + 5;
    }
    

    std::cout << "samplePoint = "<< samplePoint <<std::endl;
    // int samplePoint = 5;  // 采样点（0～9）
    int sampleInterval = 9;  // 采样间隔

    // sample_again: std::cout << "******sample_again   "<<sample_point<<"   次"<<std::endl;

    std::vector<int> BitVector {};
    int pxielVal;
    for(int i = samplePoint; i <= row.cols; i = i + sampleInterval) {
        pxielVal = row.at<uchar>(i);
        if (pxielVal == 255) {
            pxielVal = 1;
        }
        // std::cout << "pxielVal= "<< pxielVal <<std::endl;
        BitVector.push_back(pxielVal);//数据采样
    }

    return  cv::Mat(BitVector, true).t();  // 根据文档这里是一列n行，所以进行转置
}

/* -------------------【 将像素列解码为数位 】----------------
功能：
    将输入的已由列矩阵转置为行矩阵的像素列解码为数位
输入数据类型：
    cv::Mat row 已由列矩阵转置为行矩阵的像素列
输出数据类型：
    cv::Mat row 已由列矩阵转置为行矩阵的数位，由vector通过Mat(BitVector, true).t()生成
------------------------------------------------------------*/
cv::Mat convertPxielRowToBit(cv::Mat row) {
    // row =  (cv::Mat_<uchar>(1, 18) << 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0); // 测试用例
    // cout << "row = "<< row <<endl;

    // 将中间列像素计数连续相同像素，并转义，例如001100001111转义为2244
    std::vector<int> SamePxielCount {};
    int pxielCount = 0;
    cv::MatIterator_<uchar> start, it, end;
    for( it = row.begin<uchar>(), end = row.end<uchar>(), start = it; it != end; it++) {
        if (*start != *it) {
            std::cout << "pxielCount = "<< pxielCount <<std::endl;
            SamePxielCount.push_back(pxielCount);
            pxielCount = 1;
            start = it;
        } else {
            pxielCount++;
        }
    }
    // 对最后一个像素特殊处理，因为不能适用前面的条件判断
    pxielCount++;
    std::cout << "pxielCount = "<< pxielCount <<std::endl;
    SamePxielCount.push_back(pxielCount);

    int bit;
    // 对SamePxielCount成员进行排序，排除较小的异常值
    std::vector<int> SamePxielCountCopy = SamePxielCount;

    // 对转义数组进行排序
    std::sort(SamePxielCountCopy.begin(), SamePxielCountCopy.end());
    // 获取转义数组中的第五小的值，因为最小的前几个值均可能是不完整条纹，以中庸之道，取这个值
    bit = SamePxielCountCopy.at(4);
    std::cout << "bit = "<< bit <<std::endl;

    // // 获取转义数组中的最小值，即为一个字节所对应的像素
    // bit = *std::min_element(SamePxielCount.begin(), SamePxielCount.end());
    // bit = 10;
    // std::cout << "bit = "<< bit <<std::endl;

    // 将转义数组再转为数据位数组
    std::vector<int> BitVector {};
    pxielCount = 0;
    int sameBitRange;

    // 识别图像第一个像素的高低电平，转化为数据位，高电平即位1
    int pxielFlag;
    if (*row.begin<uchar>() == 255 || *row.begin<uchar>() == 1) {
        pxielFlag = 1;
    } else {
        pxielFlag = *row.begin<uchar>();  // 获取第一个像素
    }

    for (pxielCount = 0; pxielCount < SamePxielCount.size(); pxielCount++) {

        sameBitRange = round(static_cast<double>(SamePxielCount.at(pxielCount)) / bit);
        for (int bitCount = 0; bitCount < sameBitRange; bitCount ++) {
            BitVector.push_back(pxielFlag);
            // 在Bit末尾插入sameBitRaneg个数的像素，像素数值由pxielFlag决定
        }
        pxielFlag = !pxielFlag;
        // 一轮填入完成后对像素标志取反，因为转义数组的相邻两个成员指代的数据位总是反的
    }

    // cout << "Bit = "<< Mat(BitVector, true).t() <<endl;
    return  cv::Mat(BitVector, true).t();  // 根据文档这里是一列n行，所以进行转置
}

/* -------------------【 消息数据获取 】----------------
功能：
    输入待识别的已解码LED灯数位和字节头矩阵，输出数据节矩阵，例如：
    cv::Mat msgDate = getMsgDate(imageLED, msgHeaderStampTest);
输入数据类型：
    const cv::Mat imageLED 待识别的LED灯图像
    cv::Mat headerStamp 字节头矩阵，注意，此参数仅接收CV_8U格式的cv::Mat_<uchar>(i, j)的一维矩阵，
        输入示例 cv::Mat msgHeaderStampTest = (cv::Mat_<uchar>(1, 5) <<  0, 1, 0, 1, 0);
输出数据类型：
    正常情况 CV_8U格式的行矩阵
        正常输出示例 msgDate = [  0,   0,   1,   1,   0,   0,   1,   0,   1,   1,   1]
    异常情况 0矩阵，引发异常的原因包括：检测到的消息头区域重叠造成colRange提取消息区域出错；
        检测到最后一个消息头区域或者没有检测到消息头区域造成vector.at出现数组越界出错。
        处理第一种异常会通过goto迭代继续检测后面的部分直到越界成为第二种情况；处理第二种异
        常直接返回输出报错0矩阵。
        异常输出示例 msgDate = [  0]
--------------------------------------------------------*/
cv::Mat getMsgDate(cv::Mat imgRow, cv::Mat headerStamp) {
// cv::Mat getMsgDate(const cv::Mat imageLED) {
    // https://stackoverflow.com/questions/32737420/multiple-results-in-opencvsharp3-matchtemplate
    // 将获取的数据位矩阵作为待匹配矩阵
    // // 图像预处理，获取图像每行均值并插值，以行矩阵输出
    // imageLED = ImagePreProcessing(imageLED, 20);

    // // 将获取的图像每行均值进行二值化，以行矩阵输出
    // cv::Mat row =  LEDMeanRowThreshold(imageLED);

    // std::cout << "imgRow = "<< imgRow <<std::endl;
    // cv::Mat ref = convertPxielRowToBitBySample(imgRow);
    cv::Mat ref = imgRow;
    ref.convertTo(ref, CV_8U);
    std::cout << "Bit = "<< ref.t() <<std::endl;

    // cv::cvtColor(headerStamp,headerStamp,cv::COLOR_BGR2GRAY);
    // cv::Mat headerStamp = (cv::Mat_<uchar>(1, 3) << 0, 1, 0);

    // 用模板匹配寻找数据位中的消息头
    std::vector<int> HeaderStamp {};
    cv::Mat res(ref.rows - headerStamp.rows + 1, ref.cols-headerStamp.cols + 1, CV_8U);
    cv::matchTemplate(ref, headerStamp, res, CV_TM_CCOEFF_NORMED);
    cv::threshold(res, res, 0.8, 1., CV_THRESH_TOZERO);

    while (true) {
        double minval, maxval, threshold = 0.8;
        cv::Point minloc, maxloc;
        cv::minMaxLoc(res, &minval, &maxval, &minloc, &maxloc);

        if (maxval >= threshold) {
            HeaderStamp.push_back(maxloc.x);
            // 漫水填充已经识别到的区域
            cv::floodFill(res, maxloc, cv::Scalar(0), 0, cv::Scalar(.1), cv::Scalar(1.));
        } else {
            break;
        }
    }
    // 对HeaderStamp成员进行排序，否则可能出现顺序倒挂进而影响数据识别
    std::sort(HeaderStamp.begin(), HeaderStamp.end());

    // 在两个消息头之间提取ROI区域，即位ID信息
    int ptrHeaderStamp = 0;
    getROI:
    try {
        return ref.colRange(HeaderStamp.at(ptrHeaderStamp) + headerStamp.size().width,
                            HeaderStamp.at(ptrHeaderStamp + 1));
    } catch ( cv::Exception& e ) {  // 异常处理
        ptrHeaderStamp++;
        // const char* err_msg = e.what();
        // std::cout << "exception caught: " << err_msg << std::endl;
        std::cout << "正常现象，切勿惊慌" << std::endl;
        goto getROI;
    } catch ( std::out_of_range& e ) {  // 异常处理
        std::cout << "此LED图像ID无法识别" << std::endl;
        return cv::Mat_<uchar>(1, 1) << 0;
    }
}


/* -------------------【 消息处理 】----------------
功能：
    输入待识别的LED灯图像和字节头矩阵，输出数据节矩阵，例如：
    cv::Mat msgDate = getMsgDate(imageLED, msgHeaderStampTest);
输入数据类型：
    const cv::Mat imageLED 待识别的LED灯图像
    cv::Mat headerStamp 字节头矩阵，注意，此参数仅接收CV_8U格式的cv::Mat_<uchar>(i, j)的一维矩阵，
        输入示例 cv::Mat msgHeaderStampTest = (cv::Mat_<uchar>(1, 5) <<  0, 1, 0, 1, 0);
输出数据类型：
    正常情况 CV_8U格式的行矩阵
        正常输出示例 msgDate = [  0,   0,   1,   1,   0,   0,   1,   0,   1,   1,   1]
    异常情况 0矩阵，引发异常的原因包括：检测到的消息头区域重叠造成colRange提取消息区域出错；
        检测到最后一个消息头区域或者没有检测到消息头区域造成vector.at出现数组越界出错。
        处理第一种异常会通过goto迭代继续检测后面的部分直到越界成为第二种情况；处理第二种异
        常直接返回输出报错0矩阵。
        异常输出示例 msgDate = [  0]
--------------------------------------------------------*/
cv::Mat MsgProcess(cv::Mat imageLED, cv::Mat headerStamp) {
    // 图像预处理，获取图像每行均值并插值，以行矩阵输出
    imageLED = ImagePreProcessing(imageLED, 20);

    // 获取波峰波谷
    // 对于2048图像，BlurSize取15
    cv::Mat NonZeroLocations = LEDMeanRowCrestsTroughs(imageLED, 15);

    // 将获取的图像每行均值进行二值化，以行矩阵输出
    //     为了补偿宽条纹处最小值为0所造成的二值化阈值偏低，故在此进行调节。
    // 此种最小值应该指定为何值，根据观察进行配置，一般的指导规则是大于去除背景
    // 的阈值（即ImagePreProcessing函数中的backgroundThreshold参数），
    // 小于或等于其它未被去除背景的区间的最小值。例如本用例中backgroundThreshold
    // 为20，其他未被去除背景的区间的最小值在90以上，故在此经过检验后确定取40
    cv::Mat imgRow =  LEDMeanRowThreshold(imageLED, NonZeroLocations, 20, 40);

    // 以采样法获得待匹配数据序列
    // cv::Mat ref = convertPxielRowToBitBySample(imgRow);

    // 以bit宽度法计算待匹配数据序列
    cv::Mat ref = convertPxielRowToBit(imgRow);
    
    // 将待识别矩阵作为模板匹配算法的待匹配模板
    cv::Mat msgDate = getMsgDate(ref, headerStamp);

    return msgDate;
}