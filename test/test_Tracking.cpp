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
#include "pyp/fmt/fmt.hpp"
#include "pyp/timer/timer.hpp"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros_msgs/include/multi_msg_wraper.hpp"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Bool.h"
using namespace std;
using namespace MCVSLAM;
BaseExtractor *extractor;

ros::Publisher resset_pub;
std_msgs::Bool reset_msg;

int idx_number_kps;
int tic = 0;
int ret = Frame::Parse("/home/wen/SLAM/MCVSLAM/config/frame.yaml");
osg_viewer viewer("/home/wen/SLAM/MCVSLAM/config/viewer.yaml");
Map _map("/home/wen/SLAM/MCVSLAM/config/system.yaml");
cv::Mat velocity;
std::queue<FrameRef> localMap;
Pinhole cam_left("/home/wen/SLAM/MCVSLAM/config/camleft.yaml"), cam_wide("/home/wen/SLAM/MCVSLAM/config/camwide.yaml");
Tracker tracker(&_map, &viewer);
void track(std::vector<cv::Mat> &imgs, double time_stamp) {
    // 1.construct Frame
    std::vector<cv::Mat> imggrays(3);

    for (int i = 0, sz = imgs.size(); i < sz; i++) {
        cv::cvtColor(imgs[i], imggrays[i], cv::COLOR_BGR2GRAY);
    }

    FrameRef cur_frame = _map.CreateFrame(imggrays[0], imggrays[1], imggrays[2], time_stamp, &cam_left, &cam_left, &cam_wide);
    MCVSLAM::Track_State state = tracker.Track(cur_frame);
    if (state == Track_State::LOST) {
        reset_msg.data = true;
        resset_pub.publish(reset_msg);
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
    resset_pub = nh.advertise<std_msgs::Bool>("capture/reset", 1);
    reset_msg.data = true;
    resset_pub.publish(reset_msg);
    // image_transport::Subscriber sub = it.subscribe("left/image", 1, imageCallback);
    //  pub = it.advertise(PedDet::global_config::gcfg.topic_oup_detection_result,);

    ros::spin();
    ros::shutdown();
    viewer.RequestStop();
    while (viewer.IsStoped()) {
        usleep(3000);
    }
    return 0;
}
