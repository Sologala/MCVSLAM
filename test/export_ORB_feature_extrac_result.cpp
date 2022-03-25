#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <unistd.h>

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#include "BaseExtractor.hpp"
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

int main(int argc, char **argv) {
    Capture::global_capture_config.Parse("../config/capture.yaml");
    // extractor = new SURF();
    extractor = new ORB("../config/extractor.yaml");
    cmdline::parser argPaser;
    argPaser.add<string>("imgpath", 'i', "image path ", false, "../data/left/1639379430420236390.png");
    argPaser.parse_check(argc, argv);

    const std::string imgpath = argPaser.get<std::string>("imgpath");

    cv::Mat img = cv::imread(imgpath);
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    Keypoints kps;
    Desps desp;
    extractor->Extract(img, kps, desp);
    cv::Mat outimg;
    cv::drawKeypoints(img, kps, outimg);
    cv::imwrite("orb.png", outimg);
    return 0;
}
