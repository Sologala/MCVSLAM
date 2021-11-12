#include "Matcher.hpp"

#include <pyp/fmt/core.h>

#include <exception>
#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <stdexcept>
namespace MCVSLAM {
Matcher& Matcher::GetInstance(MATCH_DISTANCE _dis_mode) {
    {
        static Matcher matcher_instance;
        matcher_instance.dis_mode = _dis_mode;
        return matcher_instance;
    }
}

cv::Ptr<cv::BFMatcher> matcher_L2 = cv::BFMatcher::create(cv::NormTypes::NORM_L2);
cv::Ptr<cv::BFMatcher> matcher_HAM = cv::BFMatcher::create(cv::NormTypes::NORM_HAMMING);

cv::Ptr<cv::BFMatcher> Get_BF_Matcher(Matcher::MATCH_DISTANCE dis_mode) {
    switch (dis_mode) {
        case Matcher::MATCH_DISTANCE::HAMMING:
            return matcher_HAM;
        case Matcher::MATCH_DISTANCE::NORM2:
            return matcher_L2;
        default:
            throw std::runtime_error("ERROR at getting matcher\n");
    }
}

std::vector<cv::DMatch> Matcher::BFMatch(const Keypoints& kp1, const Desps& desp1, const Keypoints& kp2, const Desps& desp2) {
    std::vector<std::vector<cv::DMatch>> res;
    Get_BF_Matcher(this->dis_mode)->knnMatch(desp1, desp2, res, 2);
    // filter_out:
    std::vector<cv::DMatch> good_res;
    for (auto v_match : res) {
        if (v_match[0].distance / v_match[1].distance < 0.6) {
            good_res.push_back(v_match[0]);
        }
    }
    return good_res;
}

std::vector<cv::DMatch> Matcher::DBowMatch(const Keypoints& kp1, const Desps& desp1, const Keypoints& kp2, const Desps& desp2) {}

}  // namespace MCVSLAM