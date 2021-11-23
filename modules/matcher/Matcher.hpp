#ifndef MATCHER_H
#define MATCHER_H
#include <opencv2/core/mat.hpp>
#include <vector>
#pragma once
#include <opencv2/core/types.hpp>

#include "../local_feature/BaseExtractor/BaseExtractor.hpp"
#define ORB_GOOD_THRESHOLD 64

namespace MCVSLAM {
enum MATCH_DISTANCE { HAMMING = 0, COS = 1, NORM2 = 2 };

static int HammingDistance(const cv::Mat& a, const cv::Mat& b);

class MatchRes : public std::vector<cv::DMatch> {
   public:
    MatchRes& FilterThreshold(const int thres_hold = 64);
};
class MatchResKnn : public std::vector<std::vector<cv::DMatch>> {
   public:
    MatchRes FilterRatio(const float ratio = 0.6);
};

class Matcher {
   public:
    static Matcher& GetInstance(MATCH_DISTANCE _dis_mode);

   private:
    Matcher(){};
    ~Matcher(){};

   public:
    MatchRes Match(cv::InputArray desp1, cv::InputArray desp2);
    MatchResKnn KnnMatch(cv::InputArray desp1, cv::InputArray desp2, int k = 2);

    std::vector<cv::DMatch> BFMatch(const Keypoints& kp1, const Desps& desp1, const Keypoints& kp2, const Desps& desp2);
    std::vector<cv::DMatch> DBowMatch(const Keypoints& kp1, const Desps& desp1, const Keypoints& kp2, const Desps& desp2);

    static MatchResKnn KnnMatch_(const std::vector<cv::Mat>& desp1, const std::vector<cv::Mat>& desp2, int k = 2, uint n_threads = 0);

   public:
    MATCH_DISTANCE dis_mode;
};
}  // namespace MCVSLAM
#endif