/*
// Copyright 2019 
// R&C Group
*/

#ifndef imageProcess_hpp_
#define imageProcess_hpp_
// -----------------------------------【头文件包含部分】---------------------------------------
//     描述：包含程序所依赖的头文件
// ----------------------------------------------------------------------------------------------

#include "vlcCommonInclude.hpp"

// -----------------------------------------------------------------------------------------------
// **********************************************************************************************
// 
//     *********************             【图像处理函数声明部分】              *******************
// 
// **********************************************************************************************
// -----------------------------------------------------------------------------------------------


double getThreshVal_Otsu_8u(const cv::Mat& _src);
void ls_LED(const cv::Mat& _img, int& X_min, int& X_max, int& Y_min, int& Y_max, cv::Mat& imgNext);
void bwareaopen(cv::Mat &data, int n);
void thinImage(cv::Mat &srcimage);

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
cv::Mat_<float>::Mat ImagePreProcessing(cv::Mat imgLED, int backgroundThreshold);

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
cv::Mat matShift(cv::Mat frame, int shiftCol, int shiftRow);

/* -------------------【 寻找波峰波谷处理 】----------------
功能：
    寻找LED行均值的波峰波谷位置坐标
输入数据类型：
    cv::Mat imgRow 已由列矩阵转置为行矩阵的像素列，注意，必须要float类型！
        因为要为了平滑的效果，需要更高精度的数据类型
输出数据类型：
    cv::Mat NonZeroLocations 波峰波谷所在的坐标
------------------------------------------------------------*/
cv::Mat LEDMeanRowCrestsTroughs(const cv::Mat_<float>::Mat imgRow, int BlurSize);

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
                            int backgroundCompensation);

/* -------------------【 将像素列解码为数位 】----------------
功能：
    将输入的已由列矩阵转置为行矩阵的像素列解码为数位
输入数据类型：
    cv::Mat row 已由列矩阵转置为行矩阵的像素列
    int samplePoint  采样点起始位置
输出数据类型：
    cv::Mat row 已由列矩阵转置为行矩阵的数位，由vector通过Mat(BitVector, true).t()生成
------------------------------------------------------------*/
cv::Mat convertPxielRowToBitBySample(cv::Mat row);

/* -------------------【 将像素列解码为数位 】----------------
功能：
    将输入的已由列矩阵转置为行矩阵的像素列解码为数位
输入数据类型：
    cv::Mat row 已由列矩阵转置为行矩阵的像素列
输出数据类型：
    cv::Mat row 已由列矩阵转置为行矩阵的数位，由vector通过Mat(BitVector, true).t()生成
------------------------------------------------------------*/
cv::Mat convertPxielRowToBit(cv::Mat col);

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
cv::Mat getMsgDate(const cv::Mat imageLED, cv::Mat headerStamp);

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
cv::Mat MsgProcess(cv::Mat imageLED, cv::Mat headerStamp);

#endif
