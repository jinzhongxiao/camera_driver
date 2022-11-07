#include <ros/ros.h>
#include <vector>
#include <fstream>
#include <chrono>
#include <string>
#include <iostream>
#include <zconf.h>
#include <csignal>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/cudawarping.hpp>
#include <std_msgs/Int64.h>

#include <sensor_msgs/image_encodings.h>
#include "camera_source.hpp"
#include "mutex_share_mem.hpp"
#include "camera_driver/ShmInfo.h"
using namespace std;
using namespace cv;
using std::string;
using namespace std::chrono;
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <jsoncpp/json/json.h>
#include "std_msgs/String.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "mivii_cam_node");
  ros::NodeHandle node("~");
  XmlRpc::XmlRpcValue params;
  std::string video_device_name("/dev/video0");
  int image_width, org_width, image_height, org_height, camera_num, use_undist, ros_topic, mem_key = 66;
  node.param("video_device", video_device_name, std::string("/dev/video0"));
  node.param("image_width", image_width, 1920);
  node.param("image_height", image_height, 1080);
  node.param("org_width", org_width, 1920);
  node.param("org_height", org_height, 1080);
  node.param("camera_num", camera_num, 8);
  node.param("use_undist", use_undist, 0);
  node.param("mem_key", mem_key, 66);
  node.param("ros_topic", ros_topic, 0);

  std::vector<std::string> param_files;
  static int fps = 0;
  static double lastTime;
  static int frameCount = 0;
  MutexShmObject shm_obj(sizeof(unsigned int) + image_width * image_height * camera_num * 3, mem_key); //共享内存对象

  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
  image_transport::ImageTransport it(node);
  std::vector<image_transport::CameraPublisher> image_pubs;
  image_pubs.resize(camera_num);

  std::vector<ros::Publisher> image_fake_pubs;
  image_fake_pubs.resize(camera_num);

  cinfo_.reset(
      new camera_info_manager::CameraInfoManager(node, video_device_name + std::to_string(0), std::string("")));

  sensor_msgs::CameraInfo camera_info;
  camera_info.width = image_width;
  camera_info.height = image_height;
  cinfo_->setCameraInfo(camera_info);
  cinfo_->setCameraName(video_device_name + std::to_string(9));
  // grab the camera info
  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
  ci->header.frame_id = "1";
  ci->header.stamp = ros::Time::now();

  int camera_no = 0;
  ros::Rate loop_rate(5);

  sscanf(video_device_name.c_str(), "/dev/video%d", &camera_no);
  ros::Publisher hardwarePub = node.advertise<std_msgs::String>("camera_hardware_info", 1000);

  for (int i = 0; i < camera_num; i++)
  {
    if (use_undist)
    {
      std::string device_s;
      node.getParam("/dev/video" + std::to_string(camera_no + i), device_s);
      param_files.push_back(device_s);
    }
    if (ros_topic)
    {
      image_pubs[i] = it.advertiseCamera("/dev/video" + std::to_string(camera_no + i) + "/image_raw", 1);
    }
    else
    {
      image_fake_pubs[i] = node.advertise<camera_driver::ShmInfo>("/dev/video" + std::to_string(camera_no + i) + "/image_raw", 1);
    }
  }

  MultiCameraSource *source = new MultiCameraSource();
  source->init(video_device_name, cv::Size(org_width, org_height), cv::Size(image_width, image_height), camera_num, param_files);
  source->startStream();

  int image_length = image_width * image_height * 3;

  unsigned char *shmdata = NULL;

  std::vector<char *> adds;
  adds.resize(camera_num);

  for (int i = 0; i < camera_num; i++)
  {

    std::string tmp = "share_mem_img_" + std::to_string(camera_no + i);
    int fd = shm_open(tmp.c_str(), O_CREAT | O_RDWR, 0666);

    ftruncate(fd, sizeof(int) + image_width * image_height * 3);
    char *p = (char *)mmap(NULL, sizeof(int) + image_width * image_height * 3, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    adds[i] = p;
  }

  lastTime = ros::Time::now().toSec();
  Json::Value root;
  while (ros::ok())
  {
    {
      ++frameCount;
      double curTime = ros::Time::now().toSec();
      if (curTime - lastTime >= 1)
      {
        fps = frameCount;
        frameCount = 0;
        lastTime = curTime;
      }

      std::string cam_name;
      if (camera_no > 1)
        cam_name = "avm_cams_info";
      else
        cam_name = "stitch_cams_info";
      root["name"] = cam_name;
      root["hz"] = fps;
      root["msg"] = "";

      root.toStyledString();
      std::string out = root.toStyledString();
      // std::cout << out << std::endl;
      std_msgs::String msg;
      msg.data = out;
      hardwarePub.publish(msg);

      vector<Mat> imgs(camera_num);
      source->capture(imgs);
      std_msgs::Header header;
      parallel_for_(Range(0, camera_num), [&](const Range &range)
                    {
        for (int i = range.start; i < range.end; i++)  //这是需要并行计算的for循环
        {

            header.stamp = ros::Time::now();
            header.frame_id = std::to_string(i);
            header.seq = 1;
          if(ros_topic){
            // ROS
            sensor_msgs::ImagePtr img_ = (cv_bridge::CvImage(header, "bgr8", imgs[i]).toImageMsg());
            // publish the image
            sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
            ci->header.frame_id = img_->header.frame_id;
            ci->header.stamp = img_->header.stamp;
            image_pubs[i].publish(*img_, *ci);
          }
          else{
            header.stamp = ros::Time::now();
            header.frame_id = std::to_string(i);
            header.seq = 1;
            camera_driver::ShmInfo msg;
            msg.width =image_width;
            msg.height = image_height;
            msg.name = "share_mem_img_" + std::to_string(camera_no + i);
            msg.header = header;
            // std::cout << image_width <<" " << image_height << " " <<std::to_string(camera_no + i)<< std::endl;
            (*(int*)adds[i])=0;
            memcpy(adds[i]+sizeof(int), imgs[i].data,image_width * image_height * 3);
            (*(int*)adds[i])=1;
            image_fake_pubs[i].publish(msg);
          }
        } });
      // loop_rate.sleep();
    }

    // else{

    //   shmdata=shm_obj.get_mem();
    //   // ROS_INFO("share mem id is %d", shm_obj.get_shm_id());
    //   if(shmdata!=NULL)
    //   {
    //     source->capture(shmdata);
    //   }
    //   shm_obj.write_ready();

    // }
    ros::spinOnce();
  }
  if (ros_topic)
    shm_obj.write_release();
  return 0;
}
