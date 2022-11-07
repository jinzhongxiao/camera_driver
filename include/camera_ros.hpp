/*
 * @Author: roger jinzhongxiao@crop.netease.com
 * @Date: 2022-05-22 17:04:29
 * @LastEditors: roger jinzhongxiao@crop.netease.com
 * @LastEditTime: 2022-05-23 14:13:29
 * @FilePath: /excavatorvision/camera_stitch_pkg/include/camera_loader.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */


#ifndef __CAMERS_ROS_HH__
#define __CAMERS_ROS_HH__
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "camera_interface.hpp"
#ifdef JETSON_GPU

#include "/opt/miivii/include/MvGmslCamera.h"
#endif


using namespace cv;
using namespace std;
template<typename T> bool getParam(const ros::NodeHandle& nh, const std::string &key, T& var)
{
    if(!nh.getParam(key, var))
    {
        ROS_ERROR_STREAM("Could not get params ' " + nh.getNamespace()+ "/" << key << "'");
        return false;
    }
    else
        return true;

}



class Camera{

public:
    Camera(const ros::NodeHandle& camera_nh_,ros::NodeHandle nh);
    std::string getCameraNs() const;
    sensor_msgs::CompressedImageConstPtr getLastImage() const;
    cv_bridge::CvImageConstPtr getLastImageCv() const;

  void startImageSubscriber();
  void stopImageSubscriber();
private:
    void imageCb(const sensor_msgs::CompressedImageConstPtr& image);

    ros::NodeHandle camera_nh_,nh;
    std::string image_topic_;
    sensor_msgs::CompressedImageConstPtr last_image_;
    mutable cv_bridge::CvImageConstPtr last_image_cv_;
    image_transport::ImageTransport it_;
    ros::Subscriber image_sub_;

};

typedef std::shared_ptr<Camera> CameraPtr ;

class CameraLoader {
public:
    CameraLoader(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    bool loadCameras(ros::NodeHandle& nh);
    bool imagesReceived() const;
    void StartImageSubscribers();
    void stopImageSubscribers();

    bool waitForImages(const ros::Duration& timeout = ros::Duration(0)) const;
    const std::vector<CameraPtr>& cameras() const;


private:
    std::vector<CameraPtr> cameras_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
};





class BagSource:  public BaseSource{
  public:
    BagSource(const ros::NodeHandle& nh, const ros::NodeHandle& pnh){
      cams_loader = new CameraLoader(nh, pnh);
      cams_loader->StartImageSubscribers();
    };


    bool capture(std::vector<cv::Mat> &Frames) override;

    ~BagSource(){
       cams_loader->stopImageSubscribers();
    }
  private:
    
    CameraLoader* cams_loader;
};
#endif