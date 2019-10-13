这是三灯定位节点，源代码存放在src文件夹中，仅publisher.cpp是有用的源代码，其它是各种测试代码

1. 使用前请务必设置好灯具坐标信息，代码段如下：
    //计算位置坐标
	//焦距
	double f = 4;
	//透镜焦点在image sensor上的位置(与图像的像素有关，此数据适用于2048x1536)
	double Center_X = 1024;
	double Center_Y = 768;
	//三个LED灯具的真实位置
	double x1 = -300;
	double y1 = -300;
	double x2 = 300;
	double y2 = -300;
	double x3 = 300;
	double y3 = 300;
    
2. 灯具条纹数目要有差别，不同灯具条纹数量至少差一条（数目要用静态图片来数，画面上条纹“飞”的速度不同，条纹数量也有可能是相同的）

3. 条纹数目最少三条以上

4. 启动该节点所使用的命令：
    rosrun vlc_locator locator
    locator    设计作为可以自动选择双灯和三灯定位，未完善。
    locator_2  双灯定位
    locator_3  三灯定位

5. 该节点有运行依赖，依赖于相机通过节点/camera/image所传递的图像，需要该节点运行后定位节点才能运行

6. 计算出的结果存入结构体 struct XYZ 中，现在为字符串输出该结构体中的数据，如果需要可以改为其它消息格式

7. 欢迎编写launch文件，将灯具坐标，节点依赖等作为变量写入launch文件中，就不需要每次改完之后重新编译一次了

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
 
 
9.使用窗口界面查看坐标图像：rosrun image_view image_view image:=/camera/image_show
