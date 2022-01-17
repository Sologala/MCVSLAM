#ifndef SYSTEM_H
#define SYSTEM_H
#include <opencv2/core/types.hpp>
#include <string>
#include <vector>

#include "Map.hpp"
#include "ORBExtractor.hpp"
#include "Pinhole.hpp"
#include "Tracker.hpp"
#include "image_transport/publisher.h"
#include "osg_viewer.hpp"
#include "ros/init.h"
#include "ros/publisher.h"
#pragma once
namespace MCVSLAM {
class System {
   public:
    System(const std::string& _config_file);
    ~System();

    void Track(const std::vector<cv::Mat>& imgs, const double time_stmap);
    void SaveTracjtory();

    void Publish_Tracjtory();
    void Publish_TimeCost();

   public:
    Tracker* tracker = nullptr;
    Map* map = nullptr;
    BaseExtractor* extractor = nullptr;
    osg_viewer* viewer = nullptr;

    Pinhole *cam_left = nullptr, *cam_right = nullptr, *cam_wide = nullptr;

   private:
    std::string config_file;
    std::string tracj_save_file;

    bool use_viewer = false;

    // ros
    ros::NodeHandle nh;
    ros::Publisher pub_tracj;
    ros::Publisher pub_time_cost;
};
}  // namespace MCVSLAM

#endif