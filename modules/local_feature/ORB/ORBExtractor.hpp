#ifndef ORBEXTRACTOR_H__
#define ORBEXTRACTOR_H__
#pragma once
#include "../BaseExtractor/BaseExtractor.hpp"
#include "orb3_extractor/ORBextractor.h"
namespace MCVSLAM {

class ORB_Config : public ExtractorConfig {
   public:
    void Parse(const std::string& config_file);
    int iniThFAST;
    int minThFAST;
};

class ORB : public MCVSLAM::BaseExtractor {
   public:
    ORB();
    ~ORB();
    int Extract(const cv::Mat img, Keypoints& kps, Desps& desps);
    ORB_SLAM3::ORBextractor* pextractor = nullptr;
    static ORB_Config global_ORB_config;
};

}  // namespace MCVSLAM
#endif