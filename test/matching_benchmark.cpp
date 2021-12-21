#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
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
#include "capture/capture.hpp"
#include "image_transport/subscriber.h"
#include "osg_viewer.hpp"
#include "pyp/cmdline/cmdline.h"
#include "pyp/fmt/fmt.hpp"
#include "pyp/timer/timer.hpp"
using namespace std;
using namespace MCVSLAM;

std::vector<shared_ptr<Capture>> caps;
std::vector<image_transport::Publisher> pubs;
BaseExtractor *extractor;

int idx_number_kps;
int tic = 0;
int ret = Frame::Parse("/home/wen/SLAM/MCVSLAM/config/frame.yaml");
Map _map("/home/wen/SLAM/MCVSLAM/config/system.yaml");
std::queue<FrameRef> frame_queue;
Pinhole cam_left("/home/wen/SLAM/MCVSLAM/config/camleft.yaml"), cam_wide("/home/wen/SLAM/MCVSLAM/config/camwide.yaml");

void track(std::vector<cv::Mat> &imgs, double time_stamp) {
    // 1.construct Frame
    std::vector<cv::Mat> imggrays(3);
    for (int i = 0, sz = imgs.size(); i < sz; i++) {
        cv::cvtColor(imgs[i], imggrays[i], cv::COLOR_BGR2GRAY);
    }
    KeyFrame cur_frame = _map.CreateFrame(imggrays[0], imggrays[1], imggrays[2], time_stamp, &cam_left, &cam_left, &cam_wide, {});

    if (frame_queue.size()) {
        KeyFrame last_frame = frame_queue.back();
        std::vector<cv::Mat> desp1 = {cur_frame->LEFT->desps.row(1)};
        std::vector<cv::Mat> desp2 = {cur_frame->LEFT->desps.row(1)};
        for (uint i = 0; i < last_frame->LEFT->desps.rows; i++) {
            desp2.push_back(last_frame->LEFT->desps.row(i));
        }
        MyTimer::Timer _("match");
        Matcher::KnnMatch(desp1, desp2);
    }

    while (frame_queue.size() > 3) {
        KeyFrame pframe = frame_queue.front();
        frame_queue.pop();
        delete pframe;
    }
    frame_queue.push(cur_frame);
}

int main(int argc, char **argv) {
    Capture::global_capture_config.Parse("../config/capture.yaml");
    // extractor = new SURF();
    extractor = new ORB("../config/extractor.yaml");
    cmdline::parser argPaser;
    argPaser.add<string>("configPath", 'c', "config Path", false, "../config/capture.yaml");
    argPaser.parse_check(argc, argv);
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

    for (uint i = 0; i < 10; i++) {
        std::vector<cv::Mat> imgs(3);
        for (int i = 0; i < Capture::global_capture_config.capture_cnt; i++) {
            bool res = caps[i]->get(imgs[i]);
        }
        track(imgs, 0.);
    }

    fmt::print("{:^20}{:^20}{:^20}\n", "item", "time(ms)", "fps");
    for (auto p : MyTimer::Timer::COUNT) {
        fmt::print("{:^20}{:^20.6f}{:^20.6f}\n", p.first, p.second.ms(), p.second.fps());
    }

    return 0;
}
