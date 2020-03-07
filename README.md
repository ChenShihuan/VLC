这个分支主要是用作多个VLC节点定位，目前是可以支持3个VLC同时进行，运行节点是publisher（publisher2, publisher3没改）
增加VLC的数目主要通过增加图像订阅器（第317、335行），图像发布器（第320行），信息发布器（第323、339行），订阅后的回调函数及传参（第331行）；修改convert_callback函数（第356行），image_process函数（第390行）
我的方法是使用ROS_NAMESPACE命名空间来识别发布不同的mvcam的机器人节点，它们的命名是/tb3_0/，/tb3_1/，/tb3_2/，比如说tb3_0的机器人要发布一个topic，那么它发布的topic名前面就带有/tb3_0/的前缀，以此达到分辨的目的。
使机器人带有命名空间，可以通过在roslaunch启动机器人的前面加上ROS_NAMESPACE=XX来实现，例如ROS_NAMESPACE=tb3_0 roslaunch/rosrun ...
