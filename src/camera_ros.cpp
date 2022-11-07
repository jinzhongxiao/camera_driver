/*
 * @Author: roger jinzhongxiao@crop.netease.com
 * @Date: 2022-05-22 16:33:23
 * @LastEditors: roger jinzhongxiao@crop.netease.com
 * @LastEditTime: 2022-05-23 15:37:38
 * @FilePath: /excavatorvision/camera_stitch_pkg/src/Camera.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "camera_ros.hpp"

Camera::Camera(const ros::NodeHandle& camera_nh, ros::NodeHandle nh) : camera_nh_(camera_nh), nh(nh), it_(camera_nh)
{
  std::string calib_file, camera_model;
  // Load parameters
  camera_nh_.param<std::string>("image_topic", image_topic_, "image");
  camera_nh_.param<std::string>("calib_file", calib_file, " ");
  std::cout << image_topic_ << " " << calib_file << std::endl;
}

void Camera::imageCb(const sensor_msgs::CompressedImageConstPtr& image)
{
  last_image_ = image;
  ROS_INFO("Image topic %s, time offset [ now() - header->time ]  is %f", image_topic_.c_str(),
           ros::Time::now().toSec() - image->header.stamp.toSec());
  last_image_cv_.reset();
}

void Camera::startImageSubscriber()
{
  image_sub_ = nh.subscribe(image_topic_, 10, &Camera::imageCb, this);
}

void Camera::stopImageSubscriber()
{
  image_sub_.shutdown();
}

std::string Camera::getCameraNs() const
{
  return camera_nh_.getNamespace();
}

sensor_msgs::CompressedImageConstPtr Camera::getLastImage() const
{
  return last_image_;
}

cv_bridge::CvImageConstPtr Camera::getLastImageCv() const
{
  // Check if value was cached
  if (!last_image_cv_ && last_image_)
  {
    try
    {
      last_image_cv_ = cv_bridge::toCvCopy(getLastImage(), "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR_STREAM("CV Bridge conversion failed: " << e.what());
    }
  }
  return last_image_cv_;
}

CameraLoader::CameraLoader(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
{
  loadCameras(pnh_);
}
bool CameraLoader::loadCameras(ros::NodeHandle& nh)
{
  std::vector<std::string> camera_namespaces;
  getParam(nh, "cameras", camera_namespaces);
  for (const std::string& ns : camera_namespaces)
  {
    ROS_INFO_STREAM("Loading camera '" << ns << "'.");
    ros::NodeHandle camera_nh(nh, ns);
    CameraPtr camera = std::make_shared<Camera>(camera_nh, nh);
    cameras_.push_back(camera);
  }
  return true;
}

void CameraLoader::StartImageSubscribers()
{
  std::cout << "camera size " << cameras_.size() << std::endl;
  for (CameraPtr& camera : cameras_)
  {
    camera->startImageSubscriber();
  }
}

void CameraLoader::stopImageSubscribers()
{
  for (CameraPtr& camera : cameras_)
  {
    camera->stopImageSubscriber();
  }
}

const std::vector<CameraPtr>& CameraLoader::cameras() const
{
  return cameras_;
}

bool CameraLoader::imagesReceived() const
{
  for (const CameraPtr& camera : cameras_)
  {
    if (!camera->getLastImage())
    {
      return false;
    }
  }
  return true;
}

bool CameraLoader::waitForImages(const ros::Duration& timeout) const
{
  ros::Rate rate(10);
  ros::Time end = ros::Time::now() + timeout;
  while (ros::ok() && (ros::Time::now() < end || timeout.toSec() == 0.0))
  {
    if (imagesReceived())
      return true;
    ros::spinOnce();
  }

  /* Failed to receive images
  for(const CameraPtr& camera: cameras_)
  {
      if(!camera->getLastImage())
          ROS_WARN_STREAM("Timed out waiting for image on topic '" << camera->getCameraNs());
  }
*/
}

bool BagSource::capture(std::vector<cv::Mat>& Frames)
{
  if (cams_loader->waitForImages(ros::Duration(0)))
  {
    for (int i = 0; i < cams_loader->cameras().size(); i++)
    {
      Frames[i] = cams_loader->cameras()[i]->getLastImageCv()->image;
    }
  }
  return true;
}
