#ifndef BASEEXTRACTOR_H
#define BASEEXTRACTOR_H
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <pyp/yaml/yaml.hpp>
#include <vector>
#pragma once
namespace MCVSLAM {
class ExtractorConfig;

class BaseExtractor {
   public:
    BaseExtractor(){};
    virtual ~BaseExtractor(){};
    virtual int Extract(const cv::Mat img, std::vector<cv::KeyPoint>& kps, cv::Mat& desps) = 0;
};

class ExtractorConfig {
   public:
    ExtractorConfig(){};
    virtual ~ExtractorConfig(){};
    void Parse(Yaml::Node& node) {
        nkeypoints = node["nkeypoints"].As<int>();
        scale_factor = node["scale_factor"].As<double>();
        nlevels = node["nlevels"].As<int>();
    }
    int nkeypoints;
    int nlevels;
    double scale_factor;
};
}  // namespace MCVSLAM
#endif