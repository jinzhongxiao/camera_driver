/*
 * @Author: roger jinzhongxiao@crop.netease.com
 * @Date: 2022-05-22 17:06:09
 * @LastEditors: roger jinzhongxiao@crop.netease.com
 * @LastEditTime: 2022-05-23 10:07:48
 * @FilePath: /excavatorvision/camera_stitch_pkg/src/camera_loader.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <assert.h>
#include "camera_interface.hpp"
#include "camera_source.hpp"
#include <boost/foreach.hpp>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <ctime>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <omp.h>
#define foreach BOOST_FOREACH

#ifdef JETSON_GPU

uint64_t timestampbefore[8] = { 0 };         /*上一次采集图像时采集时间*/
uint64_t LinuxGetFrameTimeBefore[8] = { 0 }; /*上一次采集图像时的系统时间*/

namespace ccu = cv::cuda;
using namespace std;
using namespace chrono;
static int xioctl(int fd, int request, void* arg)
{
  int status = -1;
  do
  {
    status = ioctl(fd, request, arg);
  } while ((status == -1) && EINTR == errno);
  return status;
}

inline const char* v4l2_format_str(uint32_t fmt)
{
  switch (fmt)
  {
    case V4L2_PIX_FMT_SBGGR8:
      return "SBGGR8 (V4L2_PIX_FMT_SBGGR8)";
    case V4L2_PIX_FMT_SGBRG8:
      return "SGBRG8 (V4L2_PIX_FMT_SGBRG8)";
    case V4L2_PIX_FMT_SGRBG8:
      return "SGRBG8 (V4L2_PIX_FMT_SGRBG8)";
    case V4L2_PIX_FMT_SRGGB8:
      return "SRGGB8 (V4L2_PIX_FMT_SRGGB8)";
    case V4L2_PIX_FMT_SBGGR16:
      return "SBGGR16 (V4L2_PIX_FMT_SBGGR16)";
    case V4L2_PIX_FMT_SRGGB10:
      return "SRGGB10 (V4L2_PIX_FMT_SRGGB10)";
    case V4L2_PIX_FMT_UYVY:
      return "UYVY (V4L2_PIX_FMT_UYVY)";
  }
  return "UNKNOW";
}

inline void v4l2_print_formatdesc(const v4l2_fmtdesc& desc)
{
  LOG_DEBUG("CameraV4L2 -- format #u%", desc.index);
  LOG_DEBUG("CameraV4L2 -- desc   %s", desc.description);
  LOG_DEBUG("CameraV4L2 -- flags  %s", (desc.flags == 0 ? "V4L2_FMT_FLAG_UNCOMPRESSED" : "V4L2_FMT_FLAG_COMPRESSED"));
  LOG_DEBUG("CameraV4L2 -- fourcc 0x%X %s", desc.pixelformat, v4l2_format_str(desc.pixelformat));
}

inline void v4l2_print_format(const v4l2_format& fmt, const char* text)
{
  LOG_DEBUG("CameraV4L2 -- %s", text);
  LOG_DEBUG("CameraV4L2 -- width %u", fmt.fmt.pix.width);
  LOG_DEBUG("CameraV4L2 -- height %u", fmt.fmt.pix.height);
  LOG_DEBUG("CameraV4L2 -- pitch %u", fmt.fmt.pix.bytesperline);
  LOG_DEBUG("CameraV4L2 -- size %u", fmt.fmt.pix.sizeimage);
  LOG_DEBUG("CameraV4L2 -- format 0x%X %s", fmt.fmt.pix.pixelformat, v4l2_format_str(fmt.fmt.pix.pixelformat));
  LOG_DEBUG("CameraV4L2 -- color 0x%X", fmt.fmt.pix.colorspace);
  LOG_DEBUG("CameraV4L2 -- field 0x%X", fmt.fmt.pix.field);
}
static nv_color_fmt nvcolor_fmt[] = {
  // TODO add more pixel format mapping
  { V4L2_PIX_FMT_UYVY, NvBufferColorFormat_UYVY },
  { V4L2_PIX_FMT_VYUY, NvBufferColorFormat_VYUY },
  { V4L2_PIX_FMT_YUYV, NvBufferColorFormat_YUYV },
  { V4L2_PIX_FMT_YVYU, NvBufferColorFormat_YVYU },
};

static NvBufferColorFormat get_nvbuff_color_fmt(unsigned int v4l2_pixfmt)
{
  unsigned i;

  for (i = 0; i < sizeof(nvcolor_fmt) / sizeof(nvcolor_fmt[0]); i++)
  {
    if (v4l2_pixfmt == nvcolor_fmt[i].v4l2_pixfmt)
      return nvcolor_fmt[i].nvbuff_color;
  }

  return NvBufferColorFormat_Invalid;
}
// ------------------------SECTION--------------------------
// ---

bool CameraSource::init(const std::string& devicePath_)
{
  if (!devicePath_.empty())
    device_path_ = devicePath_;

  if (device_path_.empty())
    return false;

  if ((fd = open(device_path_.c_str(), O_RDWR, 0)) < 0)
  {
    LOG_ERROR("Camera device [%s] open failed with fd val %d", device_path_.c_str(), fd);
    assert(0);
    return false;
  }

  return initCaps() && initFormats() && initStream() && initCuda();
}

bool CameraSource::deinit()
{
  stopStream();
  deinitMMap();

  ::close(fd);
  fd = -1;

  deinitCuda();

  return true;
}

bool CameraSource::initCaps()
{
  assert(fd > 0);
  v4l2_capability caps;
  if (xioctl(fd, VIDIOC_QUERYCAP, &caps) < 0)
  {
    LOG_ERROR("CameraV4L2 -- failed to query caps (xioctl VIDIOC_QUERYCAP) for %s", device_path_.c_str());
    assert(0);
    return false;
  }
  capabilities_ = caps.capabilities;

#define PRINT_CAP(x) printf("v4l2 -- %-18s %s\n", #x, (caps.capabilities & x) ? "yes" : "no")

  PRINT_CAP(V4L2_CAP_VIDEO_CAPTURE);
  PRINT_CAP(V4L2_CAP_READWRITE);
  PRINT_CAP(V4L2_CAP_ASYNCIO);
  PRINT_CAP(V4L2_CAP_STREAMING);

  if (!(caps.capabilities & V4L2_CAP_VIDEO_CAPTURE))
  {
    LOG_ERROR("CameraV4L2 -- %s is not a video capture device", device_path_.c_str());
    assert(0);
    return false;
  }
#undef PRINT_CAP
  return true;
}

bool CameraSource::initFormats()
{
  assert(fd > 0);

  v4l2_fmtdesc desc;
  std::memset(&desc, 0, sizeof(v4l2_fmtdesc));
  desc.index = 0;
  desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  while (ioctl(fd, VIDIOC_ENUM_FMT, &desc) == 0)
  {
    formats_.push_back(desc);
    v4l2_print_formatdesc(desc);
    ++desc.index;
  }
  return true;
}

bool CameraSource::initStream()
{
  struct v4l2_format fmt;

  // if (fd == -1)
  //     LOG_ERROR("Failed to open camera device %s: %s (%d)",
  //             ctx->cam_devname, strerror(errno), errno);

  // Set camera output format
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = frame_size_.width;
  fmt.fmt.pix.height = frame_size_.height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
  v4l2_print_format(fmt, "setting new format...");
  if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
    LOG_ERROR("Failed to set camera output format: %s (%d)", strerror(errno), errno);
  // Get the real format in case the desired is not supported
  memset(&fmt, 0, sizeof fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0)
    LOG_ERROR("Failed to get camera output format: %s (%d)", strerror(errno), errno);

  v4l2_print_format(fmt, "confirmed new format");

  frame_size_.width = fmt.fmt.pix.width;
  frame_size_.height = fmt.fmt.pix.height;
  const auto pitch = fmt.fmt.pix.bytesperline;
  const auto depth = (pitch * 8) / frame_size_.width;

  if (!initMMap())
    return false;

  return true;
}

bool CameraSource::initCuda()
{
  // check unified memory support
  if (cuda_zero_copy_)
  {
    cudaDeviceProp devProp;
    cudaGetDeviceProperties(&devProp, 0);
    if (!devProp.managedMemory)
    {
      LOG_ERROR("CUDA device does not support managed memory");
      cuda_zero_copy_ = false;
    }
  }

  // allocate output buffer
  size_t size = frame_size_.width * frame_size_.height * 3;
  if (cuda_zero_copy_)
    cudaMallocManaged(&cuda_out_buffer_, size, cudaMemAttachGlobal);
  else
    cuda_out_buffer_ = (uchar*)malloc(size);

  cudaDeviceSynchronize();

  return true;
}

void CameraSource::deinitCuda()
{
  if (cuda_zero_copy_)
    cudaFree(cuda_out_buffer_);
  else
    free(cuda_out_buffer_);
}

bool CameraSource::requestCameraBuff()
{
  // Request camera v4l2 buffer
  struct v4l2_requestbuffers rb;
  memset(&rb, 0, sizeof(rb));
  rb.count = V4L2_BUFFERS_NUM;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_DMABUF;
  if (ioctl(fd, VIDIOC_REQBUFS, &rb) < 0)
    LOG_ERROR("Failed to request v4l2 buffers: %s (%d)", strerror(errno), errno);
  if (rb.count != V4L2_BUFFERS_NUM)
    LOG_ERROR("V4l2 buffer number is not as desired");

  for (unsigned int index = 0; index < V4L2_BUFFERS_NUM; index++)
  {
    struct v4l2_buffer buf;

    // Query camera v4l2 buf length
    memset(&buf, 0, sizeof buf);
    buf.index = index;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;

    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0)
      LOG_ERROR("Failed to query buff: %s (%d)", strerror(errno), errno);

    // TODO add support for multi-planer
    // Enqueue empty v4l2 buff into camera capture plane
    buf.m.fd = (unsigned long)this->g_buff[index].dmabuff_fd;
    if (buf.length != this->g_buff[index].size)
    {
      LOG_ERROR("Camera v4l2 buf length is not expected");
      this->g_buff[index].size = buf.length;
    }
    std::cout << fd << "   " << this->g_buff[index].size << std::endl;

    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
      LOG_ERROR("Failed to enqueue buffers: %s (%d)", strerror(errno), errno);
  }

  return true;
}

bool CameraSource::initMMap()
{
  NvBufferCreateParams input_params = { 0 };
  this->g_buff = (nv_buffer*)malloc(V4L2_BUFFERS_NUM * sizeof(nv_buffer));
  if (this->g_buff == NULL)
    LOG_ERROR("Failed to allocate global buffer context");

  input_params.payloadType = NvBufferPayload_SurfArray;
  input_params.width = frame_size_.height;
  input_params.height = frame_size_.width;
  input_params.layout = NvBufferLayout_Pitch;
  // Create buffer and provide it with camera
  for (unsigned int index = 0; index < V4L2_BUFFERS_NUM; index++)
  {
    int fd_tmp;
    NvBufferParams params = { 0 };
    input_params.colorFormat = get_nvbuff_color_fmt(V4L2_PIX_FMT_UYVY);
    input_params.nvbuf_tag = NvBufferTag_CAMERA;
    if (-1 == NvBufferCreateEx(&fd_tmp, &input_params))
      LOG_ERROR("Failed to create NvBuffer");

    this->g_buff[index].dmabuff_fd = fd_tmp;

    if (-1 == NvBufferGetParams(fd_tmp, &params))
      LOG_ERROR("Failed to get NvBuffer parameters");

    // TODO add multi-planar support
    // Currently it supports only YUV422 interlaced single-planar
    if (-1 ==
        NvBufferMemMap(this->g_buff[index].dmabuff_fd, 0, NvBufferMem_Read_Write, (void**)&this->g_buff[index].start))
      LOG_ERROR("Failed to map buffer");
  }

  input_params.colorFormat = get_nvbuff_color_fmt(V4L2_PIX_FMT_YUV420M);
  input_params.nvbuf_tag = NvBufferTag_NONE;

  requestCameraBuff();

  LOG_DEBUG("CameraV4L2 -- mapped %zu capture buffers with mmap", buffers.size());

  return true;
}

void CameraSource::deinitMMap()
{
  stopStream();

  int res;
  for (auto& b : buffers)
  {
    if (b.start && (res = munmap(b.start, b.length)) != 0)
      LOG_ERROR("Unmap failed: %d", res);
    b.start = nullptr;
    b.length = 0;
  }
  buffers.clear();
}

bool CameraSource::startStream()
{
  if (stream_started_)
    return false;

  v4l2_buffer buff;
  for (size_t i = 0; i < buffers.size(); ++i)
  {
    std::memset(&buff, 0, sizeof(buff));
    buff.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buff.memory = V4L2_MEMORY_MMAP;
    buff.index = i;
    if (xioctl(fd, VIDIOC_QBUF, &buff) == -1)
    {
      LOG_ERROR("VIDIOC_QBUF");
      assert(0);
      return false;
    }
  }

  LOG_DEBUG("CameraV4L2 -- %s starting stream", device_path_.c_str());
  v4l2_buf_type _type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (xioctl(fd, VIDIOC_STREAMON, &_type) < 0)
  {
    LOG_ERROR("CameraV4L2 -- failed to start streaming (errno=%i) (%s)", errno, strerror(errno));
    assert(0);
    return false;
  }

  stream_started_ = true;

  return true;
}

bool CameraSource::stopStream()
{
  if (!stream_started_)
    return true;

  LOG_DEBUG("CameraV4L2 -- %s stopping stream", device_path_.c_str());
  v4l2_buf_type _type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (xioctl(fd, VIDIOC_STREAMOFF, &_type) < 0)
  {
    LOG_ERROR("CameraV4L2 -- failed stop streaming (error=%i) (%s)", errno, strerror(errno));
    assert(0);
  }
  stream_started_ = false;
  return true;
}

bool CameraSource::capture(size_t timeout, cv::Mat& res) const
{
  struct pollfd fds[1];
  fds[0].fd = fd;
  fds[0].events = POLLIN;
  if (!stream_started_)
  {
    LOG_WARNING("Calling capture while stream is not running");
    return false;
  }

  while (poll(fds, 1, 5000) > 0)
  {
    if (fds[0].revents & POLLIN)
    {
      struct v4l2_buffer v4l2_buf;

      // Dequeue camera buff
      memset(&v4l2_buf, 0, sizeof(v4l2_buf));
      v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      v4l2_buf.memory = V4L2_MEMORY_DMABUF;

      if (ioctl(fd, VIDIOC_DQBUF, &v4l2_buf) < 0)
        LOG_ERROR("Failed to dequeue camera buff");
      // this->mvcam_ptr->GetGmslTimeStamp(ctx_t.camera_no,timestamp);

      // CheckTimeStampLog(timestamp,ctx_t.camera_no);
      // ctx->frame++;
      printf("frame No : %d\n", 0);
      // Cache sync for VIC operation
      NvBufferMemSyncForDevice(this->g_buff[v4l2_buf.index].dmabuff_fd, 0, (void**)&this->g_buff[v4l2_buf.index].start);

      // opencv io
      auto tempMat = cv::Mat(cv::Size(1920, 1080), CV_8UC2, this->g_buff[v4l2_buf.index].start);

      if (xioctl(fd, VIDIOC_QBUF, &v4l2_buf) < 0)
      {
        LOG_ERROR("CameraV4L2 -- ioctl(VIDIOC_QBUF) failed (errno=%i) (%s)", errno, strerror(errno));
        assert(0);
        return false;
      }
    }
  }

  return true;
}

bool MultiCameraSource::init(const std::string& device_start_, const cv::Size& frame_size,const cv::Size& out_size, const int camera_num_,
                             const std::vector<std::string> param_files)
{
  bool camsOpenOk = true;
  int camera_no = 0;
  this->out_size_ = out_size;
  this->frames_.resize(camera_num_);
  this->undist_frames_.resize(camera_num_);
  frame_size_ = frame_size;
  std::cout << camera_num_ << " camera " << this->frames_.size() << std::endl;
  std::string cam_devname = "/dev/video" + std::to_string(1);
  sscanf(device_start_.c_str(), "/dev/video%d", &camera_no);
  this->camera_num_ = camera_num_;
  std::cout << this->camera_num_ << std::endl;
  
  for (int i = 0; i < this->camera_num_; i++)
    _cams.push_back(CameraSource("/dev/video" + std::to_string(camera_no + i), frame_size));
  stCameraCfgSend.async_camera_num = 0;
  stCameraCfgSend.async_freq = 0;
  stCameraCfgSend.async_camera_bit_draw = 0;
  stCameraCfgSend.sync_camera_num = 8;
  stCameraCfgSend.sync_camera_bit_draw = 0xff;
  stCameraCfgSend.sync_freq = 30;

  ROS_INFO_STREAM((int)stCameraCfgSend.async_camera_num);
  ROS_INFO_STREAM((int)stCameraCfgSend.async_freq);
  ROS_INFO_STREAM((int)stCameraCfgSend.async_camera_bit_draw);
  ROS_INFO_STREAM((int)stCameraCfgSend.sync_camera_num);
  ROS_INFO_STREAM((int)stCameraCfgSend.sync_freq);
  ROS_INFO_STREAM((int)stCameraCfgSend.sync_camera_bit_draw);
  // ROS_INFO_STREAM( output_fmt_str);

  mvcam_ptr = new miivii::MvGmslCamera(stCameraCfgSend);
  for (auto& cam : _cams)
  {
    LOG_DEBUG("Initing camera %s...", cam.device_path_.c_str());
    const auto res = cam.init();
    LOG_DEBUG("Initing camera %s %s", cam.device_path_.c_str(), res ? "OK" : "FAILED");
    camsOpenOk |= res;
  }

  if (!camsOpenOk)
    return false;

  for (int i = 0; i < 8; ++i)
    if (cudaStreamCreate(&_cudaStream[i]) != cudaError::cudaSuccess)
    {
      _cudaStream[i] = NULL;
      LOG_ERROR("SyncedCameraSource: Failed to create cuda stream");
    }

  size_t planeSize = frame_size.width * frame_size.height * sizeof(uchar);
  for (auto i = 0; i < _cams.size(); ++i)
  {
    cudaMalloc(&d_src[i], planeSize * 2);
  }

  for (size_t i = 0; i < buffs.size(); ++i)
  {
    auto& buff = buffs[i];
    std::memset(&buff, 0, sizeof(v4l2_buffer));
    buff.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buff.memory = V4L2_MEMORY_MMAP;
    CUHANDLE_ERROR(cudaStreamAttachMemAsync(_cudaStream[i], _cams[i].cuda_out_buffer_, 0, cudaMemAttachGlobal));
  }

  if (param_files.size())
  {
    undist_ = true;
    for (int i = 0; i < _cams.size(); i++)
    {
      YAML::Node yamlConfig = YAML::LoadFile(param_files[i]);
      std::vector<double> K, D;
      cv::Mat map1, map2;
      K = yamlConfig["intrinsics"].as<std::vector<double>>();
      D = yamlConfig["distortion_coeffs"].as<std::vector<double>>();
      std::vector<int> resolution = yamlConfig["resolution"].as<std::vector<int>>();
      boost::array<double, 9> K1 = { K[0], K[1], K[2], K[3], K[4], K[5], K[6], K[7], K[8] };
      Mat K_M = Mat(3, 3, CV_64F, K1.c_array());

      Mat D_M = Mat(D);
      Mat KM1;
      K_M.convertTo(KM1, CV_32FC1);
      Mat new_K = getOptimalNewCameraMatrix(KM1, D_M, frame_size_, 0);  // get rectified projection.

      cv::fisheye::initUndistortRectifyMap(K_M, D_M, Mat(), new_K, frame_size_, CV_32FC1, map1, map2);

      auto& uData = undist_frames_[i];
      uData.remapX.upload(map1);
      uData.remapY.upload(map2);
      LOG_DEBUG("Generating undistort maps for camera - %i ... OK", i);
    }
  }
    return true;
  }

  bool MultiCameraSource::startStream()
  {
    bool res = true;
    for (auto& c : _cams)
      res |= c.startStream();
    return res;
  }

  bool MultiCameraSource::stopStream()
  {
    bool res = true;
    for (auto& c : _cams)
      res |= c.stopStream();
    return res;
  }
  bool MultiCameraSource::setFrameSize(const cv::Size& size)
  {
    frame_size_ = size;
    for (auto& c : _cams)
    {
      c.stopStream();
      c.deinit();
      c.frame_size_ = size;
    }
    return true;
  }

  /*图像采集时的时间戳记录，将时间戳间隔与帧数不相符的相关时间信息打印到/tmp/cameras_sdk_demo.log文件中，每个设备生成单独log文件*/
  void CheckTimeStampLog(uint64_t timestamp, uint8_t camera_no)
  {
    uint64_t FrameInterval = 0;
    char buffer[256] = { 0 };
    uint64_t LinuxFrameInterval{};
    struct timeval cur_time;
    uint64_t LinuxGetFrameTime{};
    uint64_t time_interval{};
    uint64_t FrameTransferDelay{};
    FILE* file_diff = NULL;
    char file_name[100] = { 0 };
    if (0 == timestamp)
    {
    printf("zeros\n");
      /*camera Data is not available during camera preparation*/
      return;
    }
    FrameInterval = timestamp - timestampbefore[camera_no];
    timestampbefore[camera_no] = timestamp;
    gettimeofday(&cur_time, NULL);
    LinuxGetFrameTime = cur_time.tv_sec * 1000000000 + cur_time.tv_usec * 1000;
    LinuxFrameInterval = LinuxGetFrameTime - LinuxGetFrameTimeBefore[camera_no];
    LinuxGetFrameTimeBefore[camera_no] = LinuxGetFrameTime;
    FrameTransferDelay = LinuxGetFrameTime - timestamp;
    if (1)
      time_interval = 1000000000 / 30;
    else
    {
      ;
    }
    /*    if((FrameInterval > (time_interval + 15000000) || FrameInterval < (time_interval - 15000000))
            || (LinuxFrameInterval > (time_interval + 15000000) || LinuxFrameInterval < (time_interval - 15000000))
            || FrameTransferDelay < 70000000 || FrameTransferDelay > 90000000)
        {
            printf("camera_no==========%d\n",camera_no);
            printf("timestamp==========%ld\n",timestamp/1000000);
            printf("FrameInterva===-------------=======%ld\n",FrameInterval/1000000);
            printf("LinuxGetFrameTime======------------====%ld\n",LinuxGetFrameTime/1000000);
            printf("LinuxFrameInterval======---------------====%ld\n",LinuxFrameInterval/1000000);
            printf("FrameTransferDelay======-----------------------------------====%ld\n",FrameTransferDelay/1000000);
        }*/
    if (((FrameInterval > (time_interval + 12000000) || FrameInterval < (time_interval - 12000000))) &&
        (FrameInterval != 0))
    {
      sprintf(file_name, "/tmp/cameras_sdk_demo_video%d.log", camera_no);
      file_diff = fopen(file_name, "a+");
      sprintf(buffer,
              "Timestamp : %ld FrameInterval  :  %ld FrameTransferDelay : %ld LinuxGetFrameTime : %ld "
              "LinuxFrameInterval "
              ": %ld\n",
              timestamp, FrameInterval, FrameTransferDelay, LinuxGetFrameTime, LinuxFrameInterval);
      fwrite(buffer, sizeof(char), strlen(buffer), file_diff);
      fflush(file_diff);
      fclose(file_diff);
    }
    if (1)
    {
      printf(
          "Timestamp : %ld FrameInterval : %ld FrameTransferDelay : %ld   LinuxGetFrameTime : %ld LinuxFrameInterval : "
          "%ld\n",
          timestamp, FrameInterval, FrameTransferDelay, LinuxGetFrameTime, LinuxFrameInterval);
    }
  }

  bool MultiCameraSource::capture(unsigned char* shmdata)
  {
    this->capture();
    for (int i = 0; i < this->camera_num_; i++)
    {
      cv::Mat tmp;
      this->frames_[i].gpuFrame.download(tmp);
      memcpy(shmdata+i*tmp.rows*tmp.step,tmp.data,tmp.rows*tmp.step);

    }
  }


  bool MultiCameraSource::capture(std::vector<cv::Mat> &imgs)
  {
    this->capture();
    for (int i = 0; i < this->camera_num_; i++)
    {
      cv::Mat tmp;
      this->frames_[i].gpuFrame.download(tmp);
      imgs[i] = tmp;

    }
  }
  bool MultiCameraSource::capture()
  {
    struct pollfd fds[1];
    uint64_t y;
    uint8_t* outbuf[8];

    steady_clock::time_point time = steady_clock::now();
    for (size_t i = 0; i < _cams.size(); ++i)
    {
      // std::cout << "capture " << i << std::endl;
      int fd = _cams[i].fd;
      fds[0].fd = fd;
      fds[0].events = POLLIN;

      while (poll(fds, 1, 5000) > 0)
      {
        // duration<double> time_span = duration_cast<duration<double>>(steady_clock::now() - time);
        // time = steady_clock::now();
        if (fds[0].revents & POLLIN)
        {
          auto& v4l2_buf = buffs[i];
          // Dequeue camera buff
          memset(&v4l2_buf, 0, sizeof(v4l2_buf));
          v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          v4l2_buf.memory = V4L2_MEMORY_DMABUF;

          if (ioctl(fd, VIDIOC_DQBUF, &v4l2_buf) < 0)
            LOG_ERROR("Failed to dequeue camera buff");  // std::cout << "while poll333111 enter " << std::endl;

          unsigned char camera_no;
          std::string cam_devname = "/dev/video" + std::to_string(1);
          sscanf(cam_devname.c_str(), "/dev/video%hhu", &camera_no);
          uint64_t timestamp = 0;

          //CheckTimeStampLog(timestamp,camera_no);

          // Cache sync for VIC operation
          NvBufferMemSyncForDevice(_cams[i].g_buff[v4l2_buf.index].dmabuff_fd, 0,
                                   (void**)&_cams[i].g_buff[v4l2_buf.index].start);
        }
        else
        {
          std::cout << "not data !! " << std::endl;
        }
        // time = steady_clock::now();
        break;
      }
    }
    // std::cout << "frame size " << this->frames_.size() << std::endl;
    // do processing
#ifndef NO_OMP
#pragma omp parallel for default(none) shared(frames_)
#endif
    for (size_t i = 0; i < _cams.size(); ++i)
    {
      auto& buff = buffs[i];
      auto& dataBuffer = _cams[i].g_buff[buff.index];
      auto* cudaBuffer = _cams[i].cuda_out_buffer_;
      gpuConvertUYVY2RGB_async((uchar*)dataBuffer.start, d_src[i], cudaBuffer, frame_size_.width, frame_size_.height,
                               _cudaStream[i]);
      const auto uData = cv::cuda::GpuMat(frame_size_, CV_8UC3, cudaBuffer);
      cv::cuda::GpuMat gpuOutImage;
      cv::cuda::resize(uData, gpuOutImage, out_size_);

      if (0)
      {
        cv::cuda::remap(gpuOutImage, undist_frames_[i].undistFrame, undist_frames_[i].remapX,
                        undist_frames_[i].remapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(), cudaStreamObj);
        this->frames_[i].gpuFrame = undist_frames_[i].undistFrame;
      }
      else
      {
        this->frames_[i].gpuFrame = gpuOutImage;
      }
    }

    // enqueue buffer after processing
    for (size_t i = 0; i < _cams.size(); ++i)
    {
      auto& buff = buffs[i];
      const auto& c = _cams[i];
      auto fd = c.fd;
      if (xioctl(fd, VIDIOC_QBUF, &buff) < 0)
      {
        LOG_ERROR("ioctl(VIDIOC_QBUF) failed (errno=%i) (%s)", errno, strerror(errno));
        assert(0);
        return false;
      }
    }

#ifdef NO_COMPILE
    cudaStreamObj.waitForCompletion();
    if (_cudaStream)
      cudaStreamSynchronize(_cudaStream);

#endif
    return true;
  }
#endif
