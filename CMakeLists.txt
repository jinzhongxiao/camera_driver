cmake_minimum_required(VERSION 3.0.2)
project(camera_driver LANGUAGES CXX CUDA)
SET(CMAKE_BUILD_TYPE "Release")

FIND_PACKAGE(CUDA REQUIRED)
add_definitions("-DJETSON_GPU")
set(OpenCV_VERSION 4.5.5)
set(OpenCV_VERSION_MAJOR 4)
set(OpenCV_VERSION_MINOR 5)
set(OpenCV_VERSION_PATCH 5)
set(OpenCV_SHARED ON)
set(OpenCV_CONFIG_PATH /usr/local/share/opencv4)
set(OpenCV_INSTALL_PATH /usr/local/)
set(OpenCV_LIB_COMPONENTS opencv_calib3d;opencv_core;opencv_dnn;opencv_features2d;opencv_flann;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_shape;opencv_stitching;opencv_superres;opencv_video;opencv_videoio;opencv_videostab;opencv_viz;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_cvv;opencv_datasets;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hdf;opencv_img_hash;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_reg;opencv_rgbd;opencv_saliency;opencv_stereo;opencv_structured_light;opencv_surface_matching;opencv_text;opencv_tracking;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto)
set(OpenCV_USE_MANGLED_PATHS TRUE)
set(OpenCV_MODULES_SUFFIX )
set (OpenCV_DIR /home/nvidia/opencv/build)
message(STATUS "host processor architecture is unknown")

set(_opencv_version 4)
find_package(OpenCV 4.4)
find_package(jsoncpp)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  std_msgs
  cv_bridge
  image_transport
  camera_info_manager
  std_msgs message_generation
)
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
link_directories(/usr/local/lib)
include_directories(/usr/local/include/yaml-cpp)

SET(CUDA_NVCC_FLAGS "-arch=sm_72" CACHE STRING "nvcc flags" FORCE)
#SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
SET(LIB_TYPE STATIC) 
CUDA_ADD_LIBRARY(YUV2RGB cusrc/yuv2rgb.cu)


add_message_files(
  FILES
  ShmInfo.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)
catkin_package(CATKIN_DEPENDS message_runtime 
)
include_directories(cusrc  include
 ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
)
include_directories(${PROJECT_SOURCE_DIR}/include /usr/src/jetson_multimedia_api/samples/common/algorithm/cuda  /usr/src/jetson_multimedia_api/include)
link_directories(/opt/miivii/lib/)
include_directories(  
    /usr/src/jetson_multimedia_api/include
    /usr/local/include
    /usr/local/cuda/include
    /usr/include
    ${catkin_INCLUDE_DIRS}
)
link_directories(
    ${CUDA_INCLUDE_DIRS}
    /usr/lib/aarch64-linux-gnu/tegra 
    /usr/local/cuda/lib64
    )
ADD_LIBRARY(camera_driver src/camera_source.cpp)
ADD_DEPENDENCIES(camera_driver YUV2RGB)

TARGET_LINK_LIBRARIES(camera_driver  nvbuf_utils  EGL GLESv2 -lX11 
    -lnvjpeg -lnveglstream_camconsumer -lnvjpeg -lnvosd -ldrm 
    -lnveglstreamproducer 
    -lpthread 
    -lv4l2 mvgmslcam_noopencv.so
   ${catkin_LIBRARIES} ${OpenCV_LIBS}  
    YUV2RGB
)

add_executable(miivii_gmsl_node nodes/miivii_gmsl_node.cpp )
add_dependencies(miivii_gmsl_node camera_driver)

target_link_libraries(miivii_gmsl_node  camera_driver  ${CERES_LIBRARIES} -ljsoncpp
${catkin_LIBRARIES}  ${OpenCV_INCLUDE_DIRS})

add_executable(img_vis_node nodes/img_vis_node.cpp )
add_dependencies(img_vis_node camera_driver)

target_link_libraries(img_vis_node  camera_driver  ${CERES_LIBRARIES}
${catkin_LIBRARIES}  ${OpenCV_INCLUDE_DIRS})

install(TARGETS miivii_gmsl_node img_vis_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )


install(DIRECTORY launch/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
 PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
 PATTERN ".svn" EXCLUDE
)


