#include "System.hpp"

#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <pyp/yaml/yaml.hpp>

#include "Converter.h"
#include "Frame.hpp"
#include "Pinhole.hpp"
#include "geometry_msgs/PoseArray.h"
#include "pyp/timer/timer.hpp"
#include "std_msgs/String.h"
using namespace std;
MCVSLAM::System::System(const std::string& _config_file) : config_file(_config_file) {
    Yaml::Node root;
    Yaml::Parse(root, config_file);
    // cout << root["camera_left_config_path"].As<std::string>() << root["camera_left_config_path"].cur_folder << endl;

    use_viewer = root["use_viewer"].As<bool>();
    if (use_viewer) {
        viewer = new osg_viewer(root["viewer_config_path"].AsPath());
        viewer->Start();
        // sleep(4);
    }

    cam_left = new Pinhole(root["camera_left_config_path"].AsPath());
    cam_right = cam_left;
    cam_wide = new Pinhole(root["camera_wide_config_path"].AsPath());
    map = new Map(config_file);
    map->Run();
    tracker = new Tracker(map, viewer, config_file);
    Frame::Parse(root["frame_config_path"].AsPath());

    tracj_save_file = root["tracj_save_file"].AsPath();

    // create pose publisher
    pub_tracj = nh.advertise<geometry_msgs::PoseArray>("MCVSLAM/tracj", 1);
    pub_time_cost = nh.advertise<std_msgs::String>("MCVSLAM/Time", 1);
}

MCVSLAM::System::~System() {
    if (use_viewer) {
        viewer->RequestStop();
        while (viewer->IsStoped()) {
            usleep(3000);
        }
        delete viewer;
    }
    tracker->Clear();
    map->RequestStop();
    while (map->IsStoped() == false) {
        usleep(1);
    }
    map->Clear();
    delete cam_left;
    delete cam_wide;
    delete tracker;
    delete map;

    fmt::print("system free done!\n");
}

void MCVSLAM::System::Track(const std::vector<cv::Mat>& imgs, const double time_stmap) {
    MyTimer::Timer _("Track");
    std::vector<cv::Mat> imggrays(3);
    for (int i = 0, sz = imgs.size(); i < sz; i++) {
        cv::cvtColor(imgs[i], imggrays[i], cv::COLOR_BGR2GRAY);
    }
    Track_State state = tracker->Track(imggrays[0], imggrays[1], imggrays[2], time_stmap, cam_left, cam_left, cam_wide);
}

void MCVSLAM::System::SaveTracjtory() {
    std::fstream fs(tracj_save_file, std::ios::out);
    std::vector<cv::Mat> Tcws = map->GetAllKeyFrameForShow();
    std::vector<double> time_stamps = map->GetAllKeyFrameTimeStamps();

    fmt::print("tracjtory is saved to {}\n", tracj_save_file);
    for (uint i = 0, sz = Tcws.size(); i < sz; i++) {
        cv::Mat pose = Tcws[i];
        // double time_stamp = time_stamps[i];
        double time_stamp = i;
        cv::Mat R = pose.rowRange(0, 3).colRange(0, 3);
        std::vector<float> R_q = Converter::toQuaternion(R);
        cv::Mat t = pose.rowRange(0, 3).col(3);
        std::string buffer;
        buffer =
            fmt::format("{} {} {} {} {} {} {} {}\n", time_stamp, -t.at<float>(0), t.at<float>(1), t.at<float>(2), R_q[0], R_q[1], R_q[2], R_q[3]);
        fs << buffer;
        // cout << buffer << endl;
    }
    fs.close();
}

void MCVSLAM::System::Publish_Tracjtory() {
    geometry_msgs::PoseArray msg;
    auto Tcws = map->GetAllKeyFrameForShow();
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

void MCVSLAM::System::Publish_TimeCost() {
    // fmt::print("{:^40}{:^20}{:^20}\n", "item", "time(ms)", "fps");
    // for (auto p : MyTimer::Timer::COUNT) {
    //     fmt::print("{:^40}{:^20.6f}{:^20.6f}\n", p.first, p.second.ms(), p.second.fps());
    // }
    std::vector<string> allstring;
    for (auto p : MyTimer::Timer::COUNT) {
        string temp = fmt::format("{} | {:.6f} | {:.6f}", p.first, p.second.ms(), p.second.fps());
        allstring.push_back(temp);
    }
    std::string ret;
    for (auto& s : allstring) {
        if (ret.length()) ret = ret + "\n";
        ret += s;
    }

    // publish this string
    std_msgs::String msg;
    msg.data = ret;
    pub_time_cost.publish(msg);
}
