VLC定位节点相关程序，包含以下内容：

1.mvcam：MindVisionCamera => mvcam;
    工业相机运行节点，负责拍摄图像并传输

2.vlc_locator:
    定位节点,负责处理图像并计算定位信息，定位结果为相机镜头外侧镜片中心所在的位置

详情请参看各程序包的README.md