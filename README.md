# camera_driver

V4L camera_driver on jetson platform.

## 启动摄像头

该驱动包含三路水平FOV为120的主视角摄像头和四路水平FOV为190的环视摄像头，分别对应launch文件夹中miivii_cameraHX120_xxx.launch和miivii_cameraHX190_xxx.launch。 其中，FOV为120的支持森云两种型号，分别为isx(分辨率为1920 x 1536)和imx(分辨率为1920 x 1080)两种型号。

两类摄像头支持两种方式发布图像，切换通过“ros_topic”参数：

1. 支持ros topic发布多路图像，图像topic名称对应于 “/dev/videox/image_raw”,  启动cameras.launch前将其里面的“ros_topic”参数置为“1”即可。

2. 支持 linux share memory机制，直接将同步过的多路图像数据保存在share memory中， 下游任务需要取用时直接从该内存读取图像数据即可，这样直接操作内存的方式避免了图像多次拷贝造成的时间开销。<em>roslaunch 默认启动这种方式</em>。


为了启动方便，将两类摄像头的launch合并进一个cameras.launch里, 该launch里面的参数"use_ros_topic"对应于上述说明的“ros_topic”。

### （1）同时启动所有摄像头

```roslaunch camera_driver cameras.launch```

### （2）单独启动主视角摄像头

```roslaunch camera_driver miivii_cameraHX120_imx_driver.launch```


### （2）单独启动环视摄像头

```roslaunch camera_driver miivii_cameraHX190_driver.launch```
## 使用说明

第一种ros topic启动方式可以通过订阅对应Image类型的topic来查阅图像。
第二种通过内存共享方式来发布图像，关于linux shared memory启动的具体机制详见 include 文件夹下的 mutex_share_mem.hpp 文件，里面采用 shm-id 来索引共享内存所在的位置。三路主视角拼接的shm-id默认是 66， 四路环视图像数据的shm-id 默认是67， 具体设置见  miivii_cameraHX120_xxx.launch 和 miivii_cameraHX190_xxx.launch 中的"mem_key"参数。

这种方式发布的图像不能通过常规的ROS订阅来查阅图像，需要mutex_share_mem.hpp对应的类来解析内容。具体使用方式如下：
1. 头文件包含“mutex_share_mem.hpp”和本包自定义消息提供的“ShmInfo.h”；
2. 订阅图像消息如下：
```
std::string topic_name("/dev/video0/image_raw");
ShmSubscriber* ss = new ShmSubscriber(topic_name);
ros::Subscriber sub = nh.subscribe<camera_driver::ShmInfo>(topic_name, 1, boost::bind(callback, _1, ss));
```
3. 消息回调函数：
```
void callback(const camera_driver::ShmInfo::ConstPtr& msg, ShmSubscriber* ss)
{
    cv::Mat *tmp = ss->get_img(msg);
    if(tmp)
    {
        std::cout << "I have received img" << std::endl;
        cv::imshow("img", *tmp);
        cv::waitKey(5);
    }
}

```

本包提供了一个采用共享内存访问图像的示例，在确保主视角摄像头开启后，运行：
```
rosrun camera_driver  img_vis_node
```
下游任务使用时可参考该示例代码。