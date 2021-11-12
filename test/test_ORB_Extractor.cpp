#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
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

void imageCallback(const multi_msgs::multi_imagesConstPtr &msg) {
    try {
        // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        const std::vector<cv::Mat> imgs = MsgWraper::rosMats2cvMats(*msg);
        double time_stamp = msg->header.stamp.toSec();
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
    ORB::global_ORB_config.Parse("../config/extractor.yaml");
    extractor = new ORB();
    // extractor = new SURF();

    ros::init(argc, argv, "test_node_orb_extractor");

    ros::NodeHandle nh;
    auto sub = nh.subscribe<multi_msgs::multi_images>(Capture::global_capture_config.topic, 1, imageCallback);
    // image_transport::Subscriber sub = it.subscribe("left/image", 1, imageCallback);
    //  pub = it.advertise(PedDet::global_config::gcfg.topic_oup_detection_result,);
    ros::spin();
    ros::shutdown();
    return 0;
}
