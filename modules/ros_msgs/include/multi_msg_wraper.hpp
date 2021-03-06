#ifndef MULTI_MSG_WRAPER_H
#define MULTI_MSG_WRAPER_H
// ros

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/types.hpp>

#include "multi_images.h"
#pragma once

namespace MCVSLAM {
class MsgWraper {
   public:
    static std::vector<cv::Mat> rosMats2cvMats(const multi_msgs::multi_images &multi_msg);
    static multi_msgs::multi_images cvMats2rosMats(const std::vector<cv::Mat> &imgs);

    static const sensor_msgs::ImageConstPtr cvMat2rosMat(const cv::Mat &img);
    static cv::Mat rosMat2cvMat(const sensor_msgs::ImageConstPtr &multi_msg);

};  // namespace MsgWraper
}  // namespace MCVSLAM
#endif