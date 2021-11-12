#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <deque>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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
    vector<std::vector<cv::KeyPoint>> kpss;
    vector<Desps> despss;
    for (int i = 0, sz = imgs.size(); i < sz; i++) {
        cv::cvtColor(imgs[i], imggrays[i], cv::COLOR_BGR2GRAY);
        extractor->Extract(imgs[i], kpss[i], despss[i]);
    }

    // track
    if (local_frames.size() == 0) {
        // init
    }
    // {
    //     if (q.size()) {
    //         data last = q.front();
    //         std::vector<cv::DMatch> match_res;
    //         q.pop_front();
    //         {
    //             MyTimer::Timer _("Matching");
    //             match_res = Matcher::GetInstance(Matcher::MATCH_DISTANCE::HAMMING).BFMatch(last.kps, last.desp, kps, desps);
    //         }
    //         // draw
    //         fmt::print("matched {} {} {}\n", match_res.size(), MyTimer::Timer::COUNT["Extraction"].ms(), MyTimer::Timer::COUNT["Matching"].ms());

    //         cv::Mat output_img;
    //         cv::drawMatches(last.img, last.kps, img, kps, match_res, output_img);
    //         cv::imshow("match_res", output_img);
    //     }
    //     q.push_back({img, kps, desps});
    // }

    cv::waitKey(10);
}

void imageCallback(const multi_msgs::multi_imagesConstPtr &msg) {
    try {
        // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        std::vector<cv::Mat> imgs = MsgWraper::rosMats2cvMats(*msg);
        double time_stamp = msg->header.stamp.toSec();
        // test_performance(img);
        track(imgs);
    } catch (cv_bridge::Exception &e) {
        // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    Capture::global_capture_config.Parse("../config/capture.yaml");
    ORB::global_ORB_config.Parse("../config/extractor.yaml");
    // extractor = new SURF();
    extractor = new ORB();

    ros::init(argc, argv, "test_node_orb_extractor");

    ros::NodeHandle nh;
    auto sub = nh.subscribe<multi_msgs::multi_images>(Capture::global_capture_config.topic, 1, imageCallback);
    // image_transport::Subscriber sub = it.subscribe("left/image", 1, imageCallback);
    //  pub = it.advertise(PedDet::global_config::gcfg.topic_oup_detection_result,);
    ros::spin();
    ros::shutdown();
    return 0;
}
