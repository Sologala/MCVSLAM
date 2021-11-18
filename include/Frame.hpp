#ifndef FRAME_H
#define FRAME_H
#include <memory>
#include <opencv2/core/types.hpp>
#include <unordered_set>
#include "BaseExtractor.hpp"
#include "MapPoint.hpp"
#include "ORBExtractor.hpp"
#include "Object.hpp"
#include "Pinhole.hpp"
#include "orb3_extractor/ORBextractor.h"
#pragma once
namespace MCVSLAM {
class Frame {
   public:
    Frame(cv::Mat imgleft, cv::Mat imgright, cv::Mat imgwide, double time_stamp,
            BaseCamera* cam_left, BaseCamera* cam_right, BaseCamera* cam_wide,
            float b, float bf
            );
    ~Frame();

   public:
    void ComputeStereoMatch(ObjectRef cam1, ObjectRef cam2);
    // int extractORB(ORB& obj,)
   public:

    ObjectRef LEFT, RIGHT, WIDE;
    float b, bf;

    uint fram_id;
    double time_stamp;
    std::unordered_set<MapPointRef> mps;
    std::vector<float> u_right;
    std::vector<float> depth_left;
    static ORB extractor_left, extractor_right, extractor_wide;
};
}  // namespace MCVSLAM
#endif