#ifndef MATCHER_H
#define MATCHER_H
#pragma once
#include <opencv2/core/types.hpp>

#include "../local_feature/BaseExtractor/BaseExtractor.hpp"
namespace MCVSLAM {

class Matcher {
   public:
    enum MATCH_DISTANCE { HAMMING = 0, COS = 1, NORM2 = 2 };
    static Matcher& GetInstance(MATCH_DISTANCE _dis_mode);

   private:
    Matcher(){};
    ~Matcher(){};

   public:
    std::vector<cv::DMatch> BFMatch(const Keypoints& kp1, const Desps& desp1, const Keypoints& kp2, const Desps& desp2);
    std::vector<cv::DMatch> DBowMatch(const Keypoints& kp1, const Desps& desp1, const Keypoints& kp2, const Desps& desp2);

   public:
    MATCH_DISTANCE dis_mode;
};
}  // namespace MCVSLAM
#endif