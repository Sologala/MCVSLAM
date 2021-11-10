#include "SURFExtractor.hpp"

#include <exception>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <pyp/fmt/fmt.hpp>
#include <pyp/yaml/yaml.hpp>

namespace MCVSLAM {

SURF_Config SURF::global_SURF_config = SURF_Config();

void SURF_Config::Parse(const std::string& config_file) {
    Yaml::Node root;
    Yaml::Parse(root, config_file);
    ExtractorConfig::Parse(root);
    iniThFAST = root["SURFextractor.iniThFAST"].As<int>();
    minThFAST = root["SURFextractor.minThFAST"].As<int>();
}

SURF::SURF() { pextractor = cv::xfeatures2d::SURF::create(700, 4, 3); }

SURF::~SURF() {}

int SURF::Extract(const cv::Mat img, std::vector<cv::KeyPoint>& kps, cv::Mat& desps) {
    std::vector<int> lap = {0, 0};
    int cnt;
    try {
        pextractor->detectAndCompute(img, cv::Mat(), kps, desps);
        cnt = kps.size();
    } catch (std::exception e) {
        fmt::print("SURF Extractor throw a exception");
        while (1)
            ;
    }
    return cnt;
}
}  // namespace MCVSLAM