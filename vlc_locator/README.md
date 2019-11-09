这是三灯定位节点，源代码存放在src文件夹中，publisher.cpp是各定位节点源代码，其他是所依赖的函数

1. 使用前请务必设置好相机和灯具坐标信息，代码段如下：

    以下位于vlcCommonInclude.hpp：

    //焦距
    #define focalLength 1.5

    //经过校正的相机中心
    #define centerXofImage 630.4
    #define centerYofImage 525.6
    #define centerXofImageMax 1002.5
    #define centerYofImageMax 852.5

    以下位于各定位节点源代码publisher.cpp：

    struct position P1 = {	// LED 序号
		1,		// ID_max,最大条纹数目 
		1,		// ID_min，最小条纹数目
		-470,	// LED灯具的真实位置,x坐标
		940,	// LED灯具的真实位置,y坐标
	};



    
2. 灯具条纹数目要有差别，不同灯具条纹数量至少差一条（数目要用静态图片来数，画面上条纹“飞”的速度不同，条纹数量也有可能是相同的）

3. 启动该节点所使用的命令：
    rosrun vlc_locator locator
    locator    可以自动选择双灯和三灯定位。
    locator_2  双灯定位
    locator_3  三灯定位

4. 该节点有运行依赖，依赖于相机通过节点/mvcam/image所传递的图像，需要该节点运行后定位节点才能运行

5. 计算出的结果存入结构体 geometry_msgs::Point 中

6. 欢迎编写launch文件，将灯具坐标，节点依赖等作为变量写入launch文件中，就不需要每次改完之后重新编译一次了

7. 查看被接收到的图像：rosrun image_view image_view image:=/location/image_show(可选，在本机运行，为规避图像订阅器和VLC定位程序抢图片的问题，在VLC定位程序中将接收到的图像又发布了一次)

8. 关于调试：
    在没有相机的环境下，使用图片进行调试时，为了避免改动量过大，请：
    1）将下面的代码段前一行注释掉，使用后一行来读取图片（因为该函数的输入参数为传递过来的img，所以在这里进行一个“欺骗”，将输入的图片架空）
        cv::Mat grayImage = img;
        //cv::Mat grayImage = cv::imread("/home/chen/catkin_ws/src/-3030.BMP",0);
    2）运行一个图像发布节点，具体可以是诸如网络摄像头的节点等（这种程序包网络上一大堆可用的），将此代码段中的话题名称更改为所运行的图像话题的名称，或者将别人的话题名称改为camera/image，进行“欺骗性”图像输入（反正输入进来也是被架空了）
        IMAGE_LISTENER_and_LOCATOR()  
        :it_(nh_) //构造函数  
        {  
            image_sub_ = it_.subscribe("/camera/image", 1, &IMAGE_LISTENER_and_LOCATOR::convert_callback, this); //定义图象接受器，订阅话题是“camera/image”
            // 初始化输入输出窗口  
            // cv::namedWindow(INPUT);  
            // cv::namedWindow(OUTPUT);  
        }
    3）代码中有大量中间过程的图像输出代码，如需查看将其取消注释即可
 



（暂时无效）使用窗口界面查看坐标图像：rosrun image_view image_view image:=/locator/map_show
