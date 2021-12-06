#include "Matcher.hpp"

#include <opencv2/core/hal/interface.h>
#include <pyp/fmt/core.h>

#include <exception>
#include <functional>
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <queue>
#include <stdexcept>
#include <utility>

#include "Map.hpp"
using namespace std;

namespace MCVSLAM {

MatchRes& MatchRes::FilterThreshold(const int thres_hold) {
    int i = 0, j = this->size() - 1;
    while (i <= j) {
        if ((*this)[i].distance > thres_hold) {
            (*this)[i] = (*this)[j--];
        } else {
            i++;
        }
    }
    (*this).resize(i);
    // fmt::print(" << [Threshold] {}", this->size());
    return (*this);
}

MatchRes& MatchRes::Filter_GMS(const cv::Size sz1, const cv::Size sz2, const Keypoints& kps1, const Keypoints& kps2, uint threshold) {
    MatchRes ret;
    cv::xfeatures2d::matchGMS(sz1, sz2, kps1, kps2, *this, ret, false, false, threshold);
    *this = ret;
    return *this;
}

const int HISTO_LENGTH = 40;
const float HISTO_FACTOR = 1.0f / (360.0f / HISTO_LENGTH);

MatchRes& MatchRes::FilterOrientation(const Keypoints& kps1, const Keypoints& kps2) {
    std::vector<uint> rot_bins[HISTO_LENGTH];
    for (int i = 0; i < HISTO_LENGTH; i++) rot_bins[i].reserve(500);
    for (uint i = 0, sz = this->size(); i < sz; i++) {
        const cv::DMatch m = (*this)[i];
        const cv::KeyPoint& kp1 = kps1[m.queryIdx];
        const cv::KeyPoint& kp2 = kps2[m.trainIdx];
        float rot = kp1.angle - kp2.angle;
        if (rot < 0.0) rot += 360.0f;
        int bin_id = round(rot * HISTO_FACTOR);
        if (bin_id == HISTO_LENGTH) bin_id = 0;
        assert(bin_id >= 0 && bin_id < HISTO_LENGTH);
        rot_bins[bin_id].push_back(i);
    }
    sort(&rot_bins[0], &rot_bins[0] + HISTO_LENGTH, [](const vector<uint>& a, const vector<uint>& b) { return a.size() > b.size(); });
    // pick minnal 3 size()
    MatchRes ret;
    for (uint i = 0; i < 3; i++) {
        const auto& v = rot_bins[i];
        for (auto idx : v) {
            ret.push_back((*this)[idx]);
        }
    }
    *this = ret;
    // fmt::print(" << [Orientation] {}", this->size());
    return (*this);
}

MatchRes& MatchRes::FilterFMatrix(const Keypoints& kps1, const Keypoints& kps2, const cv::Mat& F12, const std::vector<float>& LevelSigma2) {
    int i = 0, j = this->size() - 1;
    while (i <= j) {
        cv::DMatch m = (*this)[i];
        cv::KeyPoint kp1 = kps1[m.queryIdx];
        cv::KeyPoint kp2 = kps2[m.trainIdx];
        if (CheckDistEpipolarLine(kp1, kp2, F12, LevelSigma2)) {
            (*this)[i] = (*this)[j--];
        } else {
            i++;
        }
    }
    (*this).resize(i);
    // fmt::print(" << [FMatrix] {}", this->size());
    return (*this);
}

MatchRes& MatchRes::Show(const std::string& wnd_name, ObjectRef& obj1, ObjectRef& obj2) {
    cv::Mat _show_img;
    cv::drawMatches(obj1->img, obj1->kps, obj2->img, obj2->kps, *this, _show_img);
    // cv::imshow(wnd_name, _show_img);
    return *this;
}

MatchRes MatchResKnn::FilterRatio(const float ratio) {
    MatchRes res;
    for (const auto& v_match : (*this)) {
        if (v_match.size() == 1) {
            res.push_back(v_match[0]);
        } else if (v_match.size() >= 2 && v_match[0].distance / v_match[1].distance <= ratio) {
            res.push_back(v_match[0]);
        }
    }
    // fmt::print(" << [Ratio] {}", res.size());
    return res;
}

cv::Ptr<cv::BFMatcher> matcher_L2 = cv::BFMatcher::create(cv::NormTypes::NORM_L2);
cv::Ptr<cv::BFMatcher> matcher_HAM = cv::BFMatcher::create(cv::NormTypes::NORM_HAMMING);

cv::Ptr<cv::BFMatcher> Get_BF_Matcher(MATCH_DISTANCE dis_mode) {
    switch (dis_mode) {
        case MATCH_DISTANCE::HAMMING:
            return matcher_HAM;
        case MATCH_DISTANCE::NORM2:
            return matcher_L2;
        default:
            throw std::runtime_error("ERROR at getting matcher\n");
    }
}
Matcher& Matcher::GetInstance(MATCH_DISTANCE _dis_mode) {
    {
        static Matcher matcher_instance;
        matcher_instance.dis_mode = _dis_mode;
        return matcher_instance;
    }
}

MatchResKnn Matcher::KnnMatch_cv(cv::InputArray desp1, cv::InputArray desp2, int k) {
    MatchResKnn res;
    Get_BF_Matcher(MATCH_DISTANCE::HAMMING)->knnMatch(desp1, desp2, res, k);
    return res;
}

MatchRes Matcher::BFMatch(const Desps& desp1, const Desps& desp2) {
    MatchRes res;
    matcher_HAM->match(desp1, desp2, res);
    return res;
}

MatchResKnn Matcher::DBowMatch(const Desps& desp1, const DBoW3::FeatureVector& bow_feat1, const Desps& desp2, const DBoW3::FeatureVector& bow_feat2) {
    MatchResKnn res;
    // res.resize(desp1.rows, std::vector<cv::DMatch>(2));
    // We perform the matching over ORB that belong to the same vocabulary node
    // (at a certain level)
    DBoW3::FeatureVector::const_iterator f1_it = bow_feat1.begin();
    DBoW3::FeatureVector::const_iterator f1_it_end = bow_feat1.end();
    DBoW3::FeatureVector::const_iterator f2_it = bow_feat2.begin();
    DBoW3::FeatureVector::const_iterator f2_it_end = bow_feat2.end();

    while (f1_it != f1_it_end && f2_it != f2_it_end) {
        if (f1_it->first == f2_it->first) {
            const vector<uint> f1_kp_idxs = f1_it->second;
            const vector<uint> f2_kp_idxs = f2_it->second;

            // TODO 使用多线程加速
            for (uint f1_kp_idx : f1_kp_idxs) {
                const cv::Mat& d1 = desp1.row(f1_kp_idx);
                uint d[2] = {999, 999};
                uint d_idx[2] = {0, 0};
                for (uint f2_kp_idx : f2_kp_idxs) {
                    const cv::Mat& d2 = desp2.row(f2_kp_idx);
                    uint dist = HammingDistance(d1, d2);
                    if (dist < d[0]) {
                        d[1] = d[0];
                        d_idx[1] = d_idx[0];
                        d[0] = dist;
                        d_idx[0] = f2_kp_idx;
                    } else if (dist < d[1]) {
                        d[1] = dist;
                        d_idx[1] = f2_kp_idx;
                    }
                }
                if (d[1] != 999) {
                    res.push_back({cv::DMatch(f1_kp_idx, d_idx[0], d[0]), cv::DMatch(f1_kp_idx, d_idx[1], d[1])});
                }
            }

            f1_it++;
            f2_it++;
        } else if (f1_it->first < f2_it->first) {
            f1_it = bow_feat1.lower_bound(f2_it->first);
        } else {
            f2_it = bow_feat2.lower_bound(f1_it->first);
        }
    }
    return res;
}

class LoopBody : public cv::ParallelLoopBody {
   public:
    LoopBody(const std::vector<cv::Mat>& _desp1, const std::vector<cv::Mat>& _desp2, std::vector<std::vector<cv::DMatch>>& res_, int k)
        : desp1(_desp1), desp2(_desp2), res(res_), k(k) {}

    virtual void operator()(const cv::Range& r) const {
        if (k != 2) {
            for (int i = r.start; i != r.end; i++)  //遍历
            {
                cv::Mat desp = desp1[i];
                priority_queue<pair<uint, uint>> heap;  // top element is bigest.
                for (int j = 0, szj = desp2.size(); j < szj; j++) {
                    uint dist = HammingDistance(desp, desp2[j]);
                    if (heap.empty() || dist < heap.top().first) {
                        heap.push(make_pair(dist, j));
                    }
                    if (heap.size() > k) {
                        heap.pop();
                    }
                }
                for (int j = k - 1; j >= 0; j--) {
                    pair<uint, uint> temp = heap.top();
                    heap.pop();
                    res[i][j] = cv::DMatch(i, temp.second, temp.first);
                }
            }
        } else {
            for (int i = r.start; i != r.end; i++)  //遍历
            {
                cv::Mat desp = desp1[i];
                uint d[2] = {999, 999};
                uint d_idx[2] = {0, 0};
                for (int j = 0, szj = desp2.size(); j < szj; j++) {
                    uint dist = HammingDistance(desp, desp2[j]);
                    if (dist < d[0]) {
                        d[1] = d[0];
                        d_idx[1] = d_idx[0];
                        d[0] = dist;
                        d_idx[0] = j;
                    } else if (dist < d[1]) {
                        d[1] = dist;
                        d_idx[1] = j;
                    }
                }
                res[i][0] = cv::DMatch(i, d_idx[0], d[0]);
                res[i][1] = cv::DMatch(i, d_idx[1], d[1]);
            }
        }
    }

   protected:
    const std::vector<cv::Mat>& desp2;
    const std::vector<cv::Mat>& desp1;
    std::vector<std::vector<cv::DMatch>>& res;
    int k = 0;
};

MatchResKnn Matcher::KnnMatch(const std::vector<cv::Mat>& desp1, const std::vector<cv::Mat>& desp2, int k, uint n_threads) {
    assert(desp1.size());
    k = min(static_cast<int>(desp2.size()), k);
    MatchResKnn ret;
    ret.resize(desp1.size(), std::vector<cv::DMatch>(k));
    LoopBody body(desp1, desp2, ret, k);
    cv::parallel_for_(cv::Range(0, static_cast<int>(desp1.size())), body);  //启动
    return ret;
}

MatchResKnn Matcher::KnnMatch(const cv::Mat& desp1, const cv::Mat& desp2, int k, uint n_threads) {
    MatchResKnn res;
    Get_BF_Matcher(MATCH_DISTANCE::HAMMING)->knnMatch(desp1, desp2, res, k);
    return res;
}

bool CheckDistEpipolarLine(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const cv::Mat& F12, const std::vector<float>& LevelSigma2) {
    // Epipolar line in second image l = x1'F12 = [a b c]
    const float a = kp1.pt.x * F12.at<float>(0, 0) + kp1.pt.y * F12.at<float>(1, 0) + F12.at<float>(2, 0);
    const float b = kp1.pt.x * F12.at<float>(0, 1) + kp1.pt.y * F12.at<float>(1, 1) + F12.at<float>(2, 1);
    const float c = kp1.pt.x * F12.at<float>(0, 2) + kp1.pt.y * F12.at<float>(1, 2) + F12.at<float>(2, 2);

    const float num = a * kp2.pt.x + b * kp2.pt.y + c;

    const float den = a * a + b * b;

    if (den == 0) return false;

    const float dsqr = num * num / den;

    return dsqr < 3.84 * LevelSigma2[kp2.octave];
}
}  // namespace MCVSLAM