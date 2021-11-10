#include "ORBExtractor.hpp"

#include <exception>
#include <pyp/fmt/fmt.hpp>
#include <pyp/yaml/yaml.hpp>

#include "orb3_extractor/ORBextractor.h"
namespace MCVSLAM {

ORB_Config ORB::global_ORB_config = ORB_Config();

void ORB_Config::Parse(const std::string& config_file) {
    Yaml::Node root;
    Yaml::Parse(root, config_file);
    ExtractorConfig::Parse(root);
    iniThFAST = root["ORBextractor.iniThFAST"].As<int>();
    minThFAST = root["ORBextractor.minThFAST"].As<int>();
}

ORB::ORB() {
    pextractor = new ORB_SLAM3::ORBextractor(global_ORB_config.nkeypoints, global_ORB_config.scale_factor, global_ORB_config.nlevels,
                                             global_ORB_config.iniThFAST, global_ORB_config.minThFAST);
}

ORB::~ORB() { delete pextractor; }

int ORB::Extract(const cv::Mat img, std::vector<cv::KeyPoint>& kps, cv::Mat& desps) {
    std::vector<int> lap = {0, 0};
    int cnt;
    try {
        cnt = (*pextractor)(img, cv::Mat(), kps, desps, lap);
    } catch (std::exception e) {
        fmt::print("ORBSLAM3 ORB Extractoro throw a exception");
        while (1)
            ;
    }
    return cnt;
}
}  // namespace MCVSLAM