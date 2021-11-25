#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

#include <deque>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <type_traits>

#include "BaseCamera.hpp"
#include "Frame.hpp"
#include "Map.hpp"
#include "MapPoint.hpp"
#include "Object.hpp"
#include "Pinhole.hpp"
#include "PoseEstimation.hpp"
#include "capture/capture.hpp"
#include "image_transport/subscriber.h"
#include "local_feature/ORB/ORBExtractor.hpp"
#include "local_feature/SURF/SURFExtractor.hpp"
#include "matcher/Matcher.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "osg_viewer.hpp"
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
FrameRef last_frame;
KeyFrame last_keyframe;
int ret = Frame::Parse("/home/wen/SLAM/MCVSLAM/config/frame.yaml");
Pinhole cam_left("/home/wen/SLAM/MCVSLAM/config/camleft.yaml"), cam_wide("/home/wen/SLAM/MCVSLAM/config/camwide.yaml");
float baseline = 0.8;
float bf = 764.324024;

osg_viewer viewer("/home/wen/SLAM/MCVSLAM/config/viewer.yaml");
Map _map;

std::queue<FrameRef> localMap;
void track(std::vector<cv::Mat> &imgs, double time_stamp) {
    // 1.construct Frame
    std::vector<cv::Mat> imggrays(3);

    for (int i = 0, sz = imgs.size(); i < sz; i++) {
        cv::cvtColor(imgs[i], imggrays[i], cv::COLOR_BGR2GRAY);
    }

    MyTimer::Timer _("frame construct");
    _.tick();
    FrameRef cur_frame = make_shared<Frame>(imggrays[0], imggrays[1], imggrays[2], time_stamp, &cam_left, &cam_left, &cam_wide);
    _.tock();

    if (last_frame == nullptr) {
        // create mappoints
        for (uint i = 0, sz = cur_frame->LEFT->size(); i < sz; i++) {
            cv::KeyPoint kp = cur_frame->LEFT->kps[i];
            float depth = cur_frame->depth_left[i];
            if (depth != -1) {
                cv::Mat mp = cur_frame->LEFT->mpCam->unproject_z(kp.pt, depth);
                // cout << mp << endl;
                auto mpr = _map.CreateMappoint(mp, cur_frame->LEFT->desps.row(i));
            }
        }
        // create keyframes

        _map.AddKeyFrame(cur_frame);
        last_keyframe = cur_frame;
    } else {
        // Track last Frame
        ObjectRef f1 = cur_frame->LEFT, f2 = cur_frame->RIGHT;
        MatchRes match_res = Matcher::KnnMatch_cv(f1->desps, f2->desps).FilterRatio(0.6);
        cv::Mat T = PoseEstimation::_2d2d(f1, f2, match_res);
        // cout << T << endl;
    }
    viewer.Draw(_map.GetAllMappointsForShow(), 0, 0, 225);
    viewer.Commit();
    last_frame = cur_frame;
    fmt::print("{:^20}{:^20}{:^20}\n", "item", "time(ms)", "fps");
    for (auto p : MyTimer::Timer::COUNT) {
        fmt::print("{:^20}{:^20.6f}{:^20.6f}\n", p.first, p.second.ms(), p.second.fps());
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr &img_left, const sensor_msgs::ImageConstPtr &img_right,
                   const sensor_msgs::ImageConstPtr &img_wide) {
    try {
        // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        std::vector<cv::Mat> imgs = {MsgWraper::rosMat2cvMat(img_left), MsgWraper::rosMat2cvMat(img_right), MsgWraper::rosMat2cvMat(img_wide)};
        double time_stamp = img_left->header.stamp.toSec();
        // test_performance(img);
        track(imgs, time_stamp);
    } catch (cv_bridge::Exception &e) {
        // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    Capture::global_capture_config.Parse("../config/capture.yaml");
    // extractor = new SURF();
    extractor = new ORB("../config/extractor.yaml");
    viewer.Start();
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
