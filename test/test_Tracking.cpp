#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <string>

#include "Converter.h"
#include "Frame.hpp"
#include "Map.hpp"
#include "MapPoint.hpp"
#include "Matcher.hpp"
#include "ORBExtractor.hpp"
#include "Pinhole.hpp"
#include "Tracker.hpp"
#include "capture/capture.hpp"
#include "geometry_msgs/PoseArray.h"
#include "image_transport/subscriber.h"
#include "osg_viewer.hpp"
#include "pyp/cmdline/cmdline.h"
#include "pyp/fmt/fmt.hpp"
#include "pyp/timer/timer.hpp"
#include "ros/init.h"
#include "ros/publisher.h"
using namespace std;
using namespace MCVSLAM;
std::vector<shared_ptr<Capture>> caps;
std::vector<image_transport::Publisher> pubs;
ros::Publisher pub_trajector;
BaseExtractor* extractor;

int idx_number_kps;
int tic = 0;
int ret = Frame::Parse("/home/wen/SLAM/MCVSLAM/config/frame.yaml");
osg_viewer viewer("/home/wen/SLAM/MCVSLAM/config/viewer.yaml");
Map _map("/home/wen/SLAM/MCVSLAM/config/system.yaml");
cv::Mat velocity;
std::queue<FrameRef> localMap;
Pinhole cam_left("/home/wen/SLAM/MCVSLAM/config/camleft.yaml"), cam_wide("/home/wen/SLAM/MCVSLAM/config/camwide.yaml");
Tracker tracker(&_map, &viewer, "/home/wen/SLAM/MCVSLAM/config/system.yaml");

void Publish_Tracjtory(ros::Publisher& pub_tracj, const std::vector<cv::Mat>& Tcws);

void track(std::vector<cv::Mat>& imgs, double time_stamp) {
    // 1.construct Frame
    std::vector<cv::Mat> imggrays(3);
    for (int i = 0, sz = imgs.size(); i < sz; i++) {
        cv::cvtColor(imgs[i], imggrays[i], cv::COLOR_BGR2GRAY);
    }
    MCVSLAM::Track_State state = tracker.Track(imggrays[0], imggrays[1], imggrays[2], time_stamp, &cam_left, &cam_left, &cam_wide);
    Publish_Tracjtory(pub_trajector, _map.GetAllKeyFrameForShow());
}

void Publish_Tracjtory(ros::Publisher& pub_tracj, const std::vector<cv::Mat>& Tcws) {
    geometry_msgs::PoseArray msg;
    for (const cv::Mat& Tcw : Tcws) {
        geometry_msgs::Pose tmp_pose;
        cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
        cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
        std::vector<float> q = Converter::toQuaternion(Rcw);
        tmp_pose.orientation.w = q[0];
        tmp_pose.orientation.x = q[1];
        tmp_pose.orientation.y = q[2];
        tmp_pose.orientation.z = q[3];
        tmp_pose.position.x = tcw.at<float>(0);
        tmp_pose.position.y = tcw.at<float>(1);
        tmp_pose.position.z = tcw.at<float>(2);
        msg.poses.push_back(tmp_pose);
    }
    msg.header.frame_id = "mos";
    pub_tracj.publish(msg);
}

int main(int argc, char** argv) {
    Capture::global_capture_config.Parse("../config/capture.yaml");
    // extractor = new SURF();
    extractor = new ORB("../config/extractor.yaml");
    viewer.Start();
    cmdline::parser argPaser;
    argPaser.add<string>("configPath", 'c', "config Path", false, "../config/capture.yaml");
    argPaser.parse_check(argc, argv);
    ros::init(argc, argv, "dasfasdf");
    ros::NodeHandle nh;

    pub_trajector = nh.advertise<geometry_msgs::PoseArray>("MCVSLAM/tracj", 1);
    image_transport::ImageTransport it(nh);
    int fps = Capture::global_capture_config.fps;
    for (int i = 0; i < Capture::global_capture_config.capture_cnt; i++) {
        const string dataPath = Capture::global_capture_config.caps[i].source;
        fmt::print("{} source : {} \n", i, dataPath);
        if (dataPath.size() == 1 && dataPath[0] != '.') {
            stringstream ss(dataPath);
            int id = 0;
            ss >> id;
            caps.emplace_back(make_shared<Capture>(id));
        } else {
            caps.emplace_back(make_shared<Capture>(dataPath));
        }
        pubs.emplace_back(it.advertise(Capture::global_capture_config.caps[i].topic, 1));
    }
    while (ros::ok()) {
        ros::Time time_stamp = ros::Time::now();
        std::vector<cv::Mat> imgs(3);
        for (int i = 0; i < Capture::global_capture_config.capture_cnt; i++) {
            bool res = caps[i]->get(imgs[i]);
            auto publisher = pubs[i];
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgs[i]).toImageMsg();
            msg->header.stamp = time_stamp;
            publisher.publish(msg);
        }
        track(imgs, time_stamp.toSec());
    }

    viewer.RequestStop();
    while (viewer.IsStoped()) {
        usleep(3000);
    }
    return 0;
}
