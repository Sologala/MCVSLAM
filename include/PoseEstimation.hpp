#ifndef POSEESTIMATION_H
#define POSEESTIMATION_H
#include <g2o/core/base_edge.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>

#include "Frame.hpp"
#include "MapPoint.hpp"
#include "Object.hpp"
#pragma once
namespace MCVSLAM {

class PoseEstimation {
   public:
    PoseEstimation(){};
    ~PoseEstimation(){};

    // estimate pose with respect to obj1 pose
    static cv::Mat _2d2d(const ObjectRef& obj1, const ObjectRef& obj2, const std::vector<cv::DMatch>& match_res, uint method = cv::FM_8POINT);
    static int PoseOptimization(const KeyFrame& frame);
    static int BoundAdjustment(const std::unordered_set<KeyFrame>& kfs, uint n_iter);
    // static cv::Mat _2d2d(ObjectRef obj1, ObjectRef obj2, const std::vector<cv::DMatch>& match_res);
};
}  // namespace MCVSLAM
#endif