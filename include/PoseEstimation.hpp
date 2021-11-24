#ifndef POSEESTIMATION_H
#define POSEESTIMATION_H
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>

#include "Object.hpp"
#include "g2o/core/base_edge.h"
#pragma once
namespace MCVSLAM {

class PoseEstimation {
   public:
    PoseEstimation(){};
    ~PoseEstimation(){};

    // estimate pose with respect to obj1 pose
    static cv::Mat _2d2d(ObjectRef obj1, ObjectRef obj2, const std::vector<cv::DMatch>& match_res, uint method = cv::FM_8POINT);
    // static cv::Mat _2d2d(ObjectRef obj1, ObjectRef obj2, const std::vector<cv::DMatch>& match_res);
};
}  // namespace MCVSLAM
#endif