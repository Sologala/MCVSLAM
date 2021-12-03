#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>

#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#include "capture.hpp"
#include "pyp/cmdline/cmdline.h"
#include "pyp/fmt/fmt.hpp"
#include "ros/time.h"
using namespace cv;
using namespace std;
using namespace MCVSLAM;

std::vector<shared_ptr<Capture>> caps;
std::vector<image_transport::Publisher> pubs;

void RessetCaptureCallBack(const std_msgs::Bool::ConstPtr& msg) {
    for (auto& cap : caps) {
        cap->reset();
    }
}
int main(int argc, char** argv) {
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

    image_transport::ImageTransport it(nh);

    // 1. prepare capture

    auto ad_reset = nh.subscribe<std_msgs::Bool>("capture/reset", 1, &RessetCaptureCallBack);

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
    //  publish topic
    ros::Rate loop_rate(fps);
    while (nh.ok()) {
        ros::Time time_stamp = ros::Time::now();

        for (int i = 0; i < Capture::global_capture_config.capture_cnt; i++) {
            auto publisher = pubs[i];
            cv::Mat img;
            bool res = caps[i]->get(img);

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
            msg->header.stamp = time_stamp;
            publisher.publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}