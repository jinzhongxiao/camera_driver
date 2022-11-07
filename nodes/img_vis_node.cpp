#include "ros/ros.h"
#include "camera_driver/ShmInfo.h"
#include "mutex_share_mem.hpp"
#include <opencv2/opencv.hpp>

#include <opencv2/core/utility.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;

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

int main(int argc, char** argv){
    ros::init(argc, argv, "img_vis_node");
    ros::NodeHandle nh("~");

    std::string topic_name("/dev/video0/image_raw");
    ShmSubscriber* ss = new ShmSubscriber(topic_name);
    ros::Subscriber sub = nh.subscribe<camera_driver::ShmInfo>(topic_name, 1, boost::bind(callback, _1, ss));
            
    ros::spin();
    return 0;
}