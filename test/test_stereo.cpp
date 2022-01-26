#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/types_c.h>
#include <ros/init.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <opencv2/imgproc.hpp>

#include "Frame.hpp"
#include "Map.hpp"
#include "MapPoint.hpp"
#include "Pinhole.hpp"
#include "System.hpp"
#include "capture/capture.hpp"
#include "pyp/cmdline//cmdline.h"
#include "pyp/fmt//fmt.hpp"
#include "pyp/yaml/yaml.hpp"
#include "ros/init.h"
#include "ros/publisher.h"
using namespace std;
using namespace MCVSLAM;

int idx_number_kps;
int tic = 0;

void Task(System* sys, vector<cv::Mat>& imgs, double time_stamp) {
    vector<cv::Mat> grayimgs;
    for (auto img : imgs) {
        cv::Mat outimg;
        cv::cvtColor(img, outimg, CV_BGR2GRAY);
        grayimgs.push_back(outimg);
    }
    FrameRef f = sys->map->CreateFrame(grayimgs[0], grayimgs[1], grayimgs[2], time_stamp, sys->cam_left, sys->cam_right, sys->cam_wide, {});
    for (int i = 0, sz = f->LEFT->size(); i < sz; i++) {
        double z = f->depth_left[i];
        if (z >= 0) {
            cv::Mat x3D = f->LEFT->mpCam->unproject_z(f->LEFT->kps[i].pt, z);
            MapPointRef pmp = sys->map->CreateMappoint(x3D, f->LEFT->desps.row(i), f->LEFT->kps[i].octave, i, CAM_NAME::L, MP_TYPE::STEREO);
            sys->map->all_mappoints.insert(pmp);
        }
    }
    sys->viewer->Draw(sys->map->GetAllMappointsForShow(CAM_NAME::L), 0, 225, 0);
    sys->viewer->Commit();
}
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
        Task(&slam, imgs, time_stamp.toSec());
        slam.Publish_Tracjtory();
        slam.Publish_TimeCost();
    }
    while (ros::isShuttingDown() == false) {
        usleep(3000);
    }
    // slam.SaveTracjtory();
    fmt::print("system done\n");
    return 0;
}
