#include "ORBExtractor.hpp"

#include <exception>
#include <pyp/fmt/fmt.hpp>
#include <pyp/yaml/yaml.hpp>

#include "orb3_extractor/ORBextractor.h"
namespace MCVSLAM {

void ORB::Parse(const std::string& config_file) {
    Yaml::Node root;
    Yaml::Parse(root, config_file);
    nfeatures = root["nkeypoints"].As<int>();
    iniThFAST = root["ORBextractor.iniThFAST"].As<int>();
    minThFAST = root["ORBextractor.minThFAST"].As<int>();
    scaleFactor = root["scale_factor"].As<float>();
    nlevels = root["nlevels"].As<int>();
}

ORB::ORB(const std::string& config_path) {
    Parse(config_path);
    init(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST);
}

ORB::~ORB() {}

int ORB::Extract(const cv::Mat img, Keypoints& kps, Desps& desps) {
    std::vector<int> lap = {0, 0};
    int cnt;
    try {
        cnt = (*this)(img, cv::Mat(), kps, desps, lap);
    } catch (std::exception e) {
        fmt::print("ORBSLAM3 ORB Extractoro throw a exception");
        while (1)
            ;
    }
    return cnt;
}
}  // namespace MCVSLAM