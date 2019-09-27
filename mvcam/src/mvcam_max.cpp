#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <boost/assign/list_of.hpp>
#include <mvsdk/CameraApi.h> // 相机SDK的API头文件，相关API的说明在该文件中可以找到
 

using namespace cv;
using namespace std;

unsigned char           * g_pRgbBuffer;     // 处理后数据缓存区

int main(int argc, char** argv)
{
    std::cout<<"相机节点开始运行"<<std::endl;
    
    int                     iCameraCounts = 1;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList;
    int                     hCamera;
    tSdkCameraCapbility     tCapability;      //设备描述信息
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    int                     fps = 120;
    int                     channel=1;        //8位灰度图像和24位彩色图像控制，1——8位灰度图像，3——24位彩色图像。
    IplImage *iplImage = NULL;//有毒吧，放着mat不用，用这玩意儿。。。。。
    // 使用IplImage的原因可能是为了可以在C中使用相同的代码。但是我们ROS所使用的
    // 数据发送函数使用的是Mat的数据类型，与IplImage可能存在不兼容的问题，保险起见，
    // 此问题暂时通过将IplImage转换为Mat来解决，因为ROS环境完全是C++，不涉及C，所
    // 以日后可以考虑使用Mat进行完全的重构。(重构了一下失败了，老子不管了～)
    int Image_size_output_flag = 1;

    
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    // bC++:用 image_transport::Publisher (API),还是用image_transport::CameraPublisher (API)？
    
    Mat frame;
    sensor_msgs::ImagePtr msg;    
    // sensor_msgs::CameraInfo cam_info_msg;
    std_msgs::Header header;
    // header.frame_id = frame_id;
    // camera_info_manager::CameraInfoManager cam_info_manager(nh, mindvision, camera_info_url);
    // Get the saved camera info if any
    // cam_info_msg = cam_info_manager.getCameraInfo();
   

    CameraSdkInit(1);
    // 枚举设备，并建立设备列表
    CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);

    // 如果没有连接设备
    if(iCameraCounts==0){
	std::cout<<"没有连接设备"<<std::endl;
        return -1;
    }

    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

    // 如果初始化失败
    if(iStatus!=CAMERA_STATUS_SUCCESS){
	std::cout<<"初始化失败"<<std::endl;
        return -1;
    }

    // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);

    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    // g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);

    /*其他的相机参数设置
    例如: 
        CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
        CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
        CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
        更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发

    !!!!!!!注意：
    曝光时间和增益等函数需要搭配自动/手动控制函数CameraSetAeState使用！！！！！！！！！！！！！！！！！史前巨坑！！！！！！！！！！！
    */

    CameraSetAeState(hCamera,FALSE);  // 警察蜀黍，是他是他就是他～！

    CameraSetAnalogGain(hCamera,100);
    CameraSetExposureTime(hCamera,200);
	CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    CameraSetMonochrome(hCamera,TRUE);
    CameraSetFrameSpeed(hCamera,2000);
    
    tSdkImageResolution sRoiResolution;
    memset(&sRoiResolution,0,sizeof(sRoiResolution));
    sRoiResolution.iIndex = 0xff; // 设置成0xff表示自定义分辨率，设置成0到N表示选择预设分辨率
    sRoiResolution.iWidth = 2048; // 1024 X 768
    sRoiResolution.iHeight = 1536; 
    sRoiResolution.uSkipMode = 0;
    // sRoiResolution.iWidth = 1024; // 1024 X 768
    // sRoiResolution.iHeight = 768; 
    // sRoiResolution.uSkipMode = 1;
    // sRoiResolution.iWidth = 512; // 512 X 386
    // sRoiResolution.iHeight = 386; 
    // sRoiResolution.uSkipMode = 4;
    sRoiResolution.iWidthFOV = 2048;
    sRoiResolution.iHeightFOV = 1536;
    sRoiResolution.iWidthZoomHd = 0;
    sRoiResolution.iHeightZoomHd = 0;
    sRoiResolution.iHOffsetFOV = 0;
    sRoiResolution.iVOffsetFOV = 0;
    sRoiResolution.iWidthZoomSw = 0;
    sRoiResolution.iHeightZoomSw = 0;
    sRoiResolution.uBinAverageMode = 0;
    sRoiResolution.uBinSumMode = 0;
    sRoiResolution.uResampleMask = 0;
    CameraSetImageResolution(hCamera,&sRoiResolution);

    
    if(tCapability.sIspCapacity.bMonoSensor){   // CAMERA_MEDIA_TYPE_MONO8和CAMERA_MEDIA_TYPE_RGB8 分别对应8位灰度图像和24位彩色图像。
            channel=1;
            CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
        }
    else{
            channel=3;
            CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
        }



    ros::Rate r(fps);
    while (nh.ok()) 
    {
            /******************************************************/ 
        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)//获得一帧图像数据。
		{
		    CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);//将获得的相机原始输出图像数据进行处理，叠加饱和度、颜色增益和校正、降噪等处理效果，最后得到RGB888格式的图像数据。
		    		    
				if (iplImage)
		        {
		            cvReleaseImageHeader(&iplImage); // 指向释放标题的指针。
		        }
		           
            iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),IPL_DEPTH_8U,channel);
            cvSetData(iplImage,g_pRgbBuffer,sFrameInfo.iWidth*channel);// 此处只是设置指针，无图像块数据拷贝，不需担心转换效率
            /******************************************************/    
            #if 0
            #else
            // resize(cvarrToMat(iplImage),frame,Size(800,600),0,0,INTER_NEAREST);
            frame = cvarrToMat(iplImage);
            
            if(Image_size_output_flag){
                cout << "工业相机运行正常！" << endl;
                cout << "图像尺寸：" << endl;
                cout << "Width : " << frame.size().width << endl;
                cout << "Height: " << frame.size().height << endl;
                Image_size_output_flag = 0;
            }


            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            #endif
            waitKey(1);
            
                        
            // 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
			// 否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
			CameraReleaseImageBuffer(hCamera,pbyBuffer);

		}

		ros::spinOnce();
		r.sleep();
    }

    CameraUnInit(hCamera);//注意，现反初始化后再free
    free(g_pRgbBuffer);

    return 0;
}


/******************************************************/
//
//          使用方法
//
//1. 打开文件夹“运行库安装”，使用sh文件安装运行库
//2. 运行节点：
//   rosrun mvcam mvcam
//3. 使用窗口界面查看图像：
//   rosrun image_view image_view image:=/camera/image
/******************************************************/

/******************************************************/
// 函数名   : CameraSetAeState
// 功能描述 : 设置相机曝光的模式。自动或者手动。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            bAeState    TRUE，使能自动曝光；FALSE，停止自动曝光。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/

/******************************************************/    
// iplImage = cvCreateImageHeader(cvSize(width,height),depth,channels);
// cvSetData(iplImage,data,step);
//    首先由cvCreateImageHeader()创建IplImage图像头，制定图像的尺寸，深度和通道数；然后由
//    cvSetData()根据 BYTE*图像数据指针设置IplImage图像头的数据，其中step指定该IplImage图像
//    每行占的字节数，对于1通道的 IPL_DEPTH_8U图像，step可以等于width。
//    当不再使用这个新图像时，要调用void cvReleaseImage( IplImage** image )将它的头和图像数据释放！
/******************************************************/        
    
/******************************************************/
// 函数名   : CameraGetImageBuffer
// 功能描述 : 获得一帧图像数据。为了提高效率，SDK在图像抓取时采用了零拷贝机制，
//        CameraGetImageBuffer实际获得是内核中的一个缓冲区地址，
//        该函数成功调用后，必须调用CameraReleaseImageBuffer释放由
//        CameraGetImageBuffer得到的缓冲区,以便让内核继续使用
//        该缓冲区。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            pFrameInfo  图像的帧头信息指针。
//            pbyBuffer   指向图像的数据的缓冲区指针。由于
//              采用了零拷贝机制来提高效率，因此
//              这里使用了一个指向指针的指针。
//            UINT wTimes 抓取图像的超时时间。单位毫秒。在
//              wTimes时间内还未获得图像，则该函数
//              会返回超时信息。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/

/******************************************************/
// 函数名   : CameraImageProcess
// 功能描述 : 将获得的相机原始输出图像数据进行处理，叠加饱和度、
//        颜色增益和校正、降噪等处理效果，最后得到RGB888
//        格式的图像数据。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pbyIn    输入图像数据的缓冲区地址，不能为NULL。
//            pbyOut   处理后图像输出的缓冲区地址，不能为NULL。
//            pFrInfo  输入图像的帧头信息，处理完成后，帧头信息
//             中的图像格式uiMediaType会随之改变。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/


/******************************************************/

// 鉴于存在录像需求，录像和录像流相关的函数我也拿出来放在这里，以便需要的时候查看API并调用

/******************************************************/

/******************************************************/
// 函数名   : CameraInitRecord
// 功能描述 : 初始化一次录像。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            iFormat   录像的格式，当前只支持不压缩和MSCV两种方式。
//              0:不压缩；1:MSCV方式压缩。
//            pcSavePath  录像文件保存的路径。
//            b2GLimit    如果为TRUE,则文件大于2G时自动分割。
//            dwQuality   录像的质量因子，越大，则质量越好。范围1到100.
//            iFrameRate  录像的帧率。建议设定的比实际采集帧率大，
//              这样就不会漏帧。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
/******************************************************/
// 函数名   : CameraStopRecord
// 功能描述 : 结束本次录像。当CameraInitRecord后，可以通过该函数
//        来结束一次录像，并完成文件保存操作。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
/******************************************************/
// 函数名   : CameraPushFrame
// 功能描述 : 将一帧数据存入录像流中。必须调用CameraInitRecord
//        才能调用该函数。CameraStopRecord调用后，不能再调用
//        该函数。由于我们的帧头信息中携带了图像采集的时间戳
//        信息，因此录像可以精准的时间同步，而不受帧率不稳定
//        的影响。
// 参数     : hCamera     相机的句柄，由CameraInit函数获得。
//            pbyImageBuffer    图像的数据缓冲区，必须是RGB格式。
//            pFrInfo           图像的帧头信息。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/