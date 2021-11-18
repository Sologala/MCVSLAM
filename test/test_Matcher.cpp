#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

#include <deque>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "Map.hpp"
#include "capture/capture.hpp"
#include "image_transport/subscriber.h"
#include "local_feature/ORB/ORBExtractor.hpp"
#include "local_feature/SURF/SURFExtractor.hpp"
#include "matcher/Matcher.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "pyp/fmt/fmt.hpp"
#include "pyp/timer/timer.hpp"
#include "ros/subscriber.h"
#include "ros_msgs/include/multi_msg_wraper.hpp"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
using namespace std;
using namespace MCVSLAM;
BaseExtractor *extractor;

int idx_number_kps;
int tic = 0;
struct Frame {
    cv::Mat img;
    vector<cv::KeyPoint> kps;
    Desps desp;
    vector<cv::Mat> mps;
};

std::deque<Frame> local_frames;

void track(std::vector<cv::Mat> &imgs) {
    // 1. 处理图像
    std::vector<cv::Mat> imggrays(3);
    vector<std::vector<cv::KeyPoint>> kpss(3);
    vector<Desps> despss(3);
    for (int i = 0, sz = imgs.size(); i < sz; i++) {
        cv::cvtColor(imgs[i], imggrays[i], cv::COLOR_BGR2GRAY);
        extractor->Extract(imggrays[i], kpss[i], despss[i]);
    }

    // track
    if (local_frames.size() == 0) {
        // init
        auto match_res = Matcher::GetInstance(MATCH_DISTANCE::HAMMING).BFMatch(kpss[0], despss[0], kpss[1], despss[1]);
        cv::Mat show_img;
        cv::drawMatches(imgs[0], kpss[0], imgs[1], kpss[1], match_res, show_img);
        cv::imshow("match_res", show_img);
        cv::waitKey(1);
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr &img_left, const sensor_msgs::ImageConstPtr &img_right,
                   const sensor_msgs::ImageConstPtr &img_wide) {
    try {
        // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        std::vector<cv::Mat> imgs = {MsgWraper::rosMat2cvMat(img_left), MsgWraper::rosMat2cvMat(img_right), MsgWraper::rosMat2cvMat(img_wide)};
        double time_stamp = img_left->header.stamp.toSec();
        // test_performance(img);
        track(imgs);
    } catch (cv_bridge::Exception &e) {
        // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    Capture::global_capture_config.Parse("../config/capture.yaml");
    // extractor = new SURF();
    extractor = new ORB("../config/extractor.yaml");

    ros::init(argc, argv, "test_node_orb_extractor");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> sub_left(nh, Capture::global_capture_config.caps[0].topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_right(nh, Capture::global_capture_config.caps[1].topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_wide(nh, Capture::global_capture_config.caps[2].topic, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync(sub_left, sub_right, sub_wide, 10);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2, _3));

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud>("showpointcloud_output", 1);

    // image_transport::Subscriber sub = it.subscribe("left/image", 1, imageCallback);
    //  pub = it.advertise(PedDet::global_config::gcfg.topic_oup_detection_result,);
    ros::spin();
    ros::shutdown();
    return 0;
}
