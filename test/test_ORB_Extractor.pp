#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "capture/capture.hpp"
#include "image_transport/subscriber.h"
#include "local_feature/ORB/ORBExtractor.hpp"
#include "local_feature/SURF/SURFExtractor.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "pyp/fmt/fmt.hpp"
#include "pyp/timer/timer.hpp"
#include "ros/subscriber.h"
#include "ros_msgs/include/multi_msg_wraper.hpp"

using namespace std;
using namespace MCVSLAM;
BaseExtractor *extractor;

int idx_number_kps;
vector<int> number_kps;
int tic = 0;

void test(std::vector<cv::Mat> imgs) {
    cv::Mat img = imgs[0];
    std::vector<cv::KeyPoint> kps;
    Desps desps;
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    {
        MyTimer::Timer _("SURF");
        extractor->Extract(img, kps, desps);
    }
    fmt::print("{} {} {}\n", kps.size(), MyTimer::Timer::COUNT["SURF"].ms(), MyTimer::Timer::COUNT["SURF"].fps());
    cv::drawKeypoints(img, kps, img, {0, 0, 255});
    cv::imshow("img", img);
    cv::waitKey(10);
}

void imageCallback(const sensor_msgs::ImageConstPtr &img_left, const sensor_msgs::ImageConstPtr &img_right,
                   const sensor_msgs::ImageConstPtr &img_wide) {
    try {
        // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        std::vector<cv::Mat> imgs = {MsgWraper::rosMat2cvMat(img_left), MsgWraper::rosMat2cvMat(img_right), MsgWraper::rosMat2cvMat(img_wide)};
        double time_stamp = img_left->header.stamp.toSec();
        // test_performance(img);
        test(imgs);
    } catch (cv_bridge::Exception &e) {
        // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    for (int i = 500; i < 3000; i += 100) {
        number_kps.push_back(i);
    }
    Capture::global_capture_config.Parse("../config/capture.yaml");
    extractor = new ORB("../config/extractor.yaml");
    // extractor = new SURF();

    ros::init(argc, argv, "test_node_orb_extractor");

    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> sub_left(nh, Capture::global_capture_config.caps[0].topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_right(nh, Capture::global_capture_config.caps[1].topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_wide(nh, Capture::global_capture_config.caps[2].topic, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync(sub_left, sub_right, sub_wide, 10);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2, _3));

    ros::spin();
    ros::shutdown();
    return 0;
}
