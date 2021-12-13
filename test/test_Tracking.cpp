#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <unistd.h>

#include <iostream>
#include <opencv2/core.hpp>
#include <string>

#include "Frame.hpp"
#include "Map.hpp"
#include "MapPoint.hpp"
#include "Matcher.hpp"
#include "ORBExtractor.hpp"
#include "Pinhole.hpp"
#include "Tracker.hpp"
#include "capture/capture.hpp"
#include "image_transport/subscriber.h"
#include "osg_viewer.hpp"
#include "pyp/cmdline/cmdline.h"
#include "pyp/fmt/fmt.hpp"
#include "pyp/timer/timer.hpp"
#include "ros/init.h"
using namespace std;
using namespace MCVSLAM;
std::vector<shared_ptr<Capture>> caps;
std::vector<image_transport::Publisher> pubs;
BaseExtractor *extractor;

int idx_number_kps;
int tic = 0;
int ret = Frame::Parse("/home/wen/SLAM/MCVSLAM/config/frame.yaml");
osg_viewer viewer("/home/wen/SLAM/MCVSLAM/config/viewer.yaml");
Map _map("/home/wen/SLAM/MCVSLAM/config/system.yaml");
cv::Mat velocity;
std::queue<FrameRef> localMap;
Pinhole cam_left("/home/wen/SLAM/MCVSLAM/config/camleft.yaml"), cam_wide("/home/wen/SLAM/MCVSLAM/config/camwide.yaml");
Tracker tracker(&_map, &viewer, "/home/wen/SLAM/MCVSLAM/config/system.yaml");
void track(std::vector<cv::Mat> &imgs, double time_stamp) {
    // 1.construct Frame
    std::vector<cv::Mat> imggrays(3);
    for (int i = 0, sz = imgs.size(); i < sz; i++) {
        cv::cvtColor(imgs[i], imggrays[i], cv::COLOR_BGR2GRAY);
    }
    FrameRef cur_frame = _map.CreateFrame(imggrays[0], imggrays[1], imggrays[2], time_stamp, &cam_left, &cam_left, &cam_wide, {});
    MCVSLAM::Track_State state = tracker.Track(cur_frame);
}

int main(int argc, char **argv) {
    Capture::global_capture_config.Parse("../config/capture.yaml");
    // extractor = new SURF();
    extractor = new ORB("../config/extractor.yaml");
    viewer.Start();
    cmdline::parser argPaser;
    argPaser.add<string>("configPath", 'c', "config Path", false, "../config/capture.yaml");
    argPaser.parse_check(argc, argv);
    ros::init(argc, argv, "dasfasdf");
    ros::NodeHandle nh;
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
