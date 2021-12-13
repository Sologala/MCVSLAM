#ifndef MATCHER_H
#define MATCHER_H
#include <opencv2/core/mat.hpp>
#include <vector>

#include "Filter.hpp"
#include "Object.hpp"
#pragma once
#include <opencv2/core/types.hpp>

#include "BaseExtractor.hpp"
#include "DBoW3.h"
#include "Map.hpp"
#define ORB_GOOD_THRESHOLD 46

namespace MCVSLAM {
enum MATCH_DISTANCE { HAMMING = 0, COS = 1, NORM2 = 2 };

static inline uint HammingDistance(const cv::Mat &a, const cv::Mat &b) {
  const int *pa = a.ptr<int32_t>();
  const int *pb = b.ptr<int32_t>();

  uint dist = 0;

  for (int i = 0; i < 8; i++, pa++, pb++) {
    unsigned int v = *pa ^ *pb;
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
}

static bool CheckDistEpipolarLine(const cv::KeyPoint &kp1,
                                  const cv::KeyPoint &kp2, const cv::Mat &F12,
                                  const std::vector<float> &LevelSigma2);

class MatchRes : public std::vector<cv::DMatch> {
public:
  MatchRes &FilterThreshold(const int thres_hold = ORB_GOOD_THRESHOLD);
  MatchRes &Filter_GMS(const cv::Size sz1, const cv::Size sz2,
                       const Keypoints &kps1, const Keypoints &kps2,
                       uint threshold = 6.f);
  MatchRes &FilterOrientation(const Keypoints &kps1, const Keypoints &kps2);
  MatchRes &FilterFMatrix(const Keypoints &kps1, const Keypoints &kps2,
                          const cv::Mat &F12,
                          const std::vector<float> &LevelSigma2);

  MatchRes &Show(const std::string &wnd_name, ObjectRef &obj1, ObjectRef &obj2);
};

class MatchResKnn : public std::vector<std::vector<cv::DMatch>> {
public:
  MatchRes FilterRatio(const float ratio = 0.6);
};

class Matcher {
public:
  static Matcher &GetInstance(MATCH_DISTANCE _dis_mode);

private:
  Matcher(){};
  ~Matcher(){};

public:
  // MatchRes Match(cv::InputArray desp1, cv::InputArray desp2);
  static MatchResKnn KnnMatch_cv(cv::InputArray desp1, cv::InputArray desp2,
                                 int k = 2);

  static MatchRes BFMatch(const Desps &desp1, const Desps &desp2);
  static MatchResKnn DBowMatch(const Desps &desp1,
                               const DBoW3::FeatureVector &bow_feat1,
                               const Desps &desp2,
                               const DBoW3::FeatureVector &bow_feat2);

  // knn match with check kp error
  static MatchResKnn KnnMatch(const std::vector<cv::KeyPoint> &kps1,
                              const std::vector<cv::Mat> &desp1,
                              const std::vector<cv::KeyPoint> &kps2,
                              const std::vector<cv::Mat> &desp2,
                              const std::vector<float> sigma2, int k = 2,
                              uint n_threads = 0);
  static MatchResKnn KnnMatch(const std::vector<cv::Mat> &desp1,
                              const std::vector<cv::Mat> &desp2, int k = 2,
                              uint n_threads = 0);
  static MatchResKnn KnnMatch(const cv::Mat &desp1, const cv::Mat &desp2,
                              int k = 2, uint n_threads = 0);

public:
  MATCH_DISTANCE dis_mode;
};
} // namespace MCVSLAM
#endif