#ifndef SURFEXTRACTOR_H__
#define SURFEXTRACTOR_H__
#include <opencv2/xfeatures2d/nonfree.hpp>
#pragma once
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "../BaseExtractor/BaseExtractor.hpp"
namespace MCVSLAM {

class SURF_Config : public ExtractorConfig {
   public:
    void Parse(const std::string& config_file);
    int iniThFAST;
    int minThFAST;
};

class SURF : public MCVSLAM::BaseExtractor {
   public:
    SURF();
    ~SURF();
    int Extract(const cv::Mat img, Keypoints& kps, Desps& desps);

    cv::Ptr<cv::xfeatures2d::SURF> pextractor;
    static SURF_Config global_SURF_config;
};

}  // namespace MCVSLAM
#endif