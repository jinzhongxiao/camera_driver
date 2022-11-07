/*
 * @Author: roger jinzhongxiao@crop.netease.com
 * @Date: 2022-05-22 17:04:29
 * @LastEditors: roger jinzhongxiao@crop.netease.com
 * @LastEditTime: 2022-05-23 14:13:29
 * @FilePath: /excavatorvision/camera_stitch_pkg/include/camera_loader.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef __CAMER_SOURCE_HH__
#define __CAMER_SOURCE_HH__
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <dlfcn.h>
#include "camera_interface.hpp"
#include <opencv2/cudawarping.hpp>
#ifdef JETSON_GPU
#include "/opt/miivii/include/MvGmslCamera.h"
#include <cuda_runtime.h>
#include "NvVideoConverter.h"
#include "NvUtils.h"
#include "NvCudaProc.h"
#include "nvbuf_utils.h"
#endif
#include "yuv2rgb.cuh"
using namespace cv;
using namespace std;

#define CAMERA_WIDTH 1920
#define CAMERA_HEIGHT 1080

#define V4L2_BUFFERS_NUM 4
#define CAM_NUMS 8

typedef unsigned char uchar;
typedef unsigned int uint;

#define CUHANDLE_ERROR(err) (cudaHandleError(err, __FILE__, __LINE__))

typedef struct _InternalCameraParams
{
  cv::Size resolution;
  std::array<double, 9> K;
  std::array<double, 14> distortion;
  cv::Size captureResolution;
  bool read(const std::string& filepath, const int camNum,
            const cv::Size& resol = cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT),
            const cv::Size& cameraResol = cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT));

private:
  int _cameraNum;
} InternalCameraParams;

typedef struct _ExternalCameraParams
{
  //
} ExternalCameraParams;

#ifdef JETSON_GPU

typedef struct
{
  // Hold the user accessible pointer
  unsigned char* start;
  // Hold the memory length
  unsigned int size;
  // Hold the file descriptor of NvBuffer
  int dmabuff_fd;
} nv_buffer;

// Correlate v4l2 pixel format and NvBuffer color format
typedef struct
{
  unsigned int v4l2_pixfmt;
  NvBufferColorFormat nvbuff_color;
} nv_color_fmt;

class CameraSource
{
private:
  typedef struct _buffer
  {
    void* start;
    size_t length;
  } buffer;

public:
  int fd = -1;
  std::string device_path_;
  uint32_t capabilities_ = 0;
  std::vector<v4l2_fmtdesc> formats_;
  cv::Size frame_size_;
  bool stream_started_ = false;
  bool cuda_zero_copy_ = true;
  uchar* cuda_out_buffer_ = nullptr;

  std::vector<buffer> buffers;
  // Global buffer ptr
  nv_buffer* g_buff;

public:
  CameraSource(const std::string& device_path = {}, const cv::Size& frame_size = {})
    : device_path_(device_path), frame_size_(frame_size)
  {
  }
  ~CameraSource()
  {
    deinit();
  }

  bool init(const std::string& device_path_ = {});
  bool deinit();
  bool requestCameraBuff();

private:
  bool initFormats();
  void deinitCuda();

  bool initMMap();
  void deinitMMap();

  bool initCaps();
  bool initStream();
  bool initCuda();

public:
  bool startStream();
  bool stopStream();
  bool capture(size_t timeout, cv::Mat& res) const;
};

class MultiCameraSource
{
public:
  typedef struct _Frame
  {
    cv::cuda::GpuMat gpuFrame;
  } Frame;
  typedef struct _CameraUndistortData
  {
    cv::cuda::GpuMat remapX, remapY;
    cv::cuda::GpuMat undistFrame;
    cv::Rect roiFrame;
  } CameraUndistortData;

  // std::array<cv::Mat, CAM_NUMS> Ks;
private:
  cv::Size frame_size_{ 1920, 1080 };
  cv::Size out_size_{ 1920, 1080 };
  std::vector<CameraSource> _cams;
  bool cuda_zero_copy_ = true;

public:
  MultiCameraSource() = default;
  ~MultiCameraSource()
  {
    close();
  }

  bool init(const std::string& device_start = {}, const cv::Size& frame_size = {}, const cv::Size& out_size = {}, const int camera_num_ = 8,
            const std::vector<std::string> param_files = {});
  bool startStream();
  bool stopStream();
  bool capture();
  bool capture(std::vector<cv::Mat> &imgs);
  bool capture(unsigned char* shmdata);
  void close()
  {
    for (auto i = 0; i < _cams.size(); ++i)
    {
      cudaFree(d_src[i]);
    }
    for (auto& cam : _cams)
      cam.stopStream();

    for (auto i = 0; i < _cams.size(); ++i)
    {
      if (_cudaStream[i])
        cudaStreamDestroy(_cudaStream[i]);
      _cudaStream[i] = NULL;
    }
  }

public:
  const CameraSource& getCamera(int index) const
  {
    return _cams[index];
  }
  size_t getCamerasCount() const
  {
    return _cams.size();
  }
  cv::Size getFramesize() const
  {
    return frame_size_;
  }
  bool setFrameSize(const cv::Size& size);

private:
  std::array<v4l2_buffer, CAM_NUMS> buffs{};
  std::array<nv_buffer, CAM_NUMS> nv_buffs{};
  cudaStream_t _cudaStream[CAM_NUMS]{ NULL };
  cv::cuda::Stream cudaStreamObj{ cv::cuda::Stream::Null() };
  uchar* d_src[CAM_NUMS];  // cuda source memory
  miivii::MvGmslCamera* mvcam_ptr;
  int camera_num_;
  bool undist_ = false;
  std::vector<Frame> frames_;
  std::vector<CameraUndistortData> undist_frames_;
  struct sync_out_a_cfg_client_t stCameraCfgSend = {};
};
#endif
#endif