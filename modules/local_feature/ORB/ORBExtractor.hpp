#ifndef ORBEXTRACTOR_H__
#define ORBEXTRACTOR_H__
#pragma once
#include "../BaseExtractor/BaseExtractor.hpp"
#include "orb3_extractor/ORBextractor.h"
namespace MCVSLAM {

class ORB : public MCVSLAM::BaseExtractor, public ORB_SLAM3::ORBextractor {
   public:
    ORB(){};
    ORB(const std::string& config_path);
    ~ORB();

    int Extract(const cv::Mat img, Keypoints& kps, Desps& desps);

   private:
    void Parse(const std::string& config_file);
};

}  // namespace MCVSLAM
#endif