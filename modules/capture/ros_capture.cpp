#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#include "capture.hpp"
#include "multi_msg_wraper.hpp"
#include "pyp/cmdline/cmdline.h"
#include "pyp/fmt/fmt.hpp"
#include "ros/publisher.h"
#include "ros/time.h"
using namespace cv;
using namespace std;
using namespace MCVSLAM;

int main(int argc, char **argv) {
    std::string configPath;
    std::string dataPath;
    cmdline::parser argPaser;
    argPaser.add<string>("configPath", 'c', "config Path", false, "../config/capture.yaml");
    argPaser.parse_check(argc, argv);
    configPath = argPaser.get<string>("configPath");
    printf("config path is %s\n", configPath.c_str());
    MCVSLAM::Capture::global_capture_config.Parse(configPath);

    ros::init(argc, argv, "capture_publisher");
    ros::NodeHandle nh;

    // 1. prepare capture
    std::vector<shared_ptr<Capture>> caps;
    ros::Publisher pub = nh.advertise<multi_msgs::multi_images>(Capture::global_capture_config.topic, 1);
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
    }
    //  publish topic
    ros::Rate loop_rate(fps);
    while (nh.ok()) {
        ros::Time time_stamp = ros::Time::now();
        std::vector<cv::Mat> imgs;
        for (int i = 0; i < Capture::global_capture_config.capture_cnt; i++) {
            cv::Mat img;
            bool res = caps[i]->get(img);
            if (res) {
                imgs.push_back(img);
            }
        }

        if (imgs.size() == Capture::global_capture_config.capture_cnt) {
            multi_msgs::multi_images msg = MsgWraper::cvMats2rosMats(imgs);
            msg.header.stamp = time_stamp;
            pub.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
