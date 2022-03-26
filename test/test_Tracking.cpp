#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pyp/fmt/color.h>
#include <ros/ros.h>
#include <unistd.h>

#include <fstream>
#include <ios>
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
#include "System.hpp"
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

int idx_numbekr_kps;
int tic = 0;
int main(int argc, char** argv) {
    std::vector<shared_ptr<Capture>> caps;
    std::vector<image_transport::Publisher> pubs;
    Capture::global_capture_config.Parse("../config/capture.yaml");
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
        pubs.push_back(it.advertise(Capture::global_capture_config.caps[i].topic, 1));
    }

    System slam("../config/system.yaml");

    while (ros::isShuttingDown() == false) {
        ros::Time time_stamp = ros::Time::now();
        std::vector<cv::Mat> imgs(3);
        int cnt = 0;
        for (int i = 0; i < Capture::global_capture_config.capture_cnt; i++) {
            int res = caps[i]->get(imgs[i]);
            if (res != 0) {
                continue;
            }
            cnt++;
            auto& publisher = pubs[i];
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgs[i]).toImageMsg();
            msg->header.stamp = time_stamp;
            publisher.publish(msg);
        }
        if (cnt == 3) {
            slam.Track(imgs, time_stamp.toSec());
            slam.Publish_Tracjtory();
            slam.Publish_TimeCost();
        } else {
            break;
        }
    }
    slam.SaveTracjtory();
    fmt::print("system done\n");
    return 0;
}
