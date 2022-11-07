/*
 * @Author: roger jinzhongxiao@crop.netease.com
 * @Date: 2022-05-22 16:34:36
 * @LastEditors: roger jinzhongxiao@crop.netease.com
 * @LastEditTime: 2022-05-22 19:58:09
 * @FilePath: /excavatorvision/camera_stitch_pkg/include/cameras.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * 
 * c
 */
#ifndef __CAMERA_INTERFACE_HH__
#define __CAMERA_INTERFACE_HH__
#include <vector>
#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#define LOG_DEBUG(msg, ...)   printf("DEBUG:   " msg "\n", ##__VA_ARGS__)
#define LOG_INFO(msg, ...)   printf("INFO:   " msg "\n", ##__VA_ARGS__)
#define LOG_WARNING(msg, ...) printf("WARNING: " msg "\n", ##__VA_ARGS__)
#define LOG_ERROR(msg, ...)   printf("ERROR:   " msg "\n", ##__VA_ARGS__)

class BaseSource{
  public:
    BaseSource(){};
    virtual bool init(int frame_rate = 30, const std::string camera_fmt_str= "UYVY", 
        const std::string output_fmt_str= "ABGR32") {
        std::cout << "Iniitalize ...... " << std::endl;
        return true;
      }
          
    virtual bool capture(std::vector<cv::Mat> &Frames) = 0;

};





#endif
