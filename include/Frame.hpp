#ifndef FRAME_H
#define FRAME_H
#include <memory>
#include <opencv2/core/types.hpp>
#include <unordered_set>

#include "BaseExtractor.hpp"
#include "ORBExtractor.hpp"
#include "Object.hpp"
#include "Pinhole.hpp"
#include "orb3_extractor/ORBextractor.h"
#pragma once
namespace MCVSLAM {
class Frame {
    friend class Map;

   private:
    Frame(cv::Mat imgleft, cv::Mat imgright, cv::Mat imgwide, double time_stamp, BaseCamera* cam_left, BaseCamera* cam_right, BaseCamera* cam_wide,
          uint _id);

   public:
    ~Frame();
    static int Parse(const std::string& config_file);

   public:
    void ComputeStereoMatch(ObjectRef cam1, ObjectRef cam2);
    // int extractORB(ORB& obj,)
    cv::Mat SetPose(cv::Mat Tcw);
    cv::Mat GetPose();

    void ComputeBow();

   public:
    ObjectRef LEFT, RIGHT, WIDE;

    uint id;
    double time_stamp;

    std::vector<float> u_right;
    std::vector<float> depth_left;
    std::vector<cv::DMatch> left_right_match_res;

    static ORB extractor_left, extractor_right, extractor_wide;
    static cv::Mat Trl;
    static cv::Mat Twl;
    static float b, bf;
};

}  // namespace MCVSLAM
#endif