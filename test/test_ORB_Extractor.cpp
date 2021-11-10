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

#include "image_transport/subscriber.h"
#include "local_feature/ORB/ORBExtractor.hpp"
#include "local_feature/SURF/SURFExtractor.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "pyp/fmt/fmt.hpp"
#include "pyp/timer/timer.hpp"
#include "ros/subscriber.h"
using namespace std;
using namespace MCVSLAM;
BaseExtractor *extractor;

int idx_number_kps;
vector<int> number_kps;
int tic = 0;

void test(cv::Mat img) {
    std::vector<cv::KeyPoint> kps;
    cv::Mat desps;
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

void test_performance(cv::Mat img) {
    std::vector<cv::KeyPoint> kps;
    cv::Mat desps;
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    if (tic % 1000 == 0 && idx_number_kps < number_kps.size()) {
        if (tic != 0) {
            fmt::print("{} {} {}\n", ORB::global_ORB_config.nkeypoints, MyTimer::Timer::COUNT["ORB"].ms(), MyTimer::Timer::COUNT["ORB"].fps());
        }

        if (extractor) {
            delete extractor;
            ORB::global_ORB_config.nkeypoints = number_kps[idx_number_kps++];
            extractor = new ORB();
        }
    }
    {
        MyTimer::Timer _("ORB");
        extractor->Extract(img, kps, desps);
        tic += 1;
    }
    // fmt::print("{}\t{}\n", kps.size(), MyTimer::Timer::COUNT["ORB"].fps());
    cv::drawKeypoints(img, kps, img, {0, 0, 255});
    cv::imshow("img", img);
    cv::waitKey(10);
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        const cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        double time_stamp = msg->header.stamp.toSec();
        // test_performance(img);
        test(img);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    for (int i = 500; i < 3000; i += 100) {
        number_kps.push_back(i);
    }

    std::string extractor_config_file = "../config/extractor.yaml";
    ORB::global_ORB_config.Parse(extractor_config_file);
    // extractor = new ORB();
    extractor = new SURF();

    ros::init(argc, argv, "test_node");

    ros::NodeHandle nh;
    // ros::Rate loop_rate(10);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("left/image", 1, imageCallback);
    //  pub = it.advertise(PedDet::global_config::gcfg.topic_oup_detection_result,);
    ros::spin();
    ros::shutdown();
    return 0;
}
