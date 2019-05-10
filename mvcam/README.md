MindVisionCamera => mvcam
这是工业相机运行节点，负责拍摄图像并传输

1. 打开文件夹“运行库大全”，根据平台选择合适的共享库放入lib文件夹中
2. 运行节点：rosrun mvcam mvcam
3. 使用窗口界面查看图像：rosrun image_view image_view image:=/camera/image
4. 如需更多功能，请参阅头文件 CameraApi.h ，里面有大量的API可供使用
5. 如果已经插上相机，而运行时提示"没有连接设备"，其原因是权限问题，运行文件夹“运行库大全”中的instal.sh，以使得相机可以在非管理员权限下运行。
