#include "Matcher.hpp"

#include <opencv2/core/hal/interface.h>
#include <pyp/fmt/core.h>

#include <exception>
#include <functional>
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <queue>
#include <stdexcept>
#include <utility>
using namespace std;

namespace MCVSLAM {

int HammingDistance(const cv::Mat& a, const cv::Mat& b) {
    const int* pa = a.ptr<int32_t>();
    const int* pb = b.ptr<int32_t>();

    int dist = 0;

    for (int i = 0; i < 8; i++, pa++, pb++) {
        unsigned int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
    // ----------------use the keypoint net feature
}

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
    return (*this);
}

MatchRes MatchResKnn::FilterRatio(const float ratio) {
    MatchRes res;
    for (auto v_match : (*this)) {
        if (v_match[0].distance / v_match[1].distance <= ratio) {
            res.push_back(v_match[0]);
        }
    }
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

MatchRes Matcher::Match(cv::InputArray desp1, cv::InputArray desp2) {
    MatchRes res;
    Get_BF_Matcher(this->dis_mode)->match(desp1, desp2, res);
    return res;
}

MatchResKnn Matcher::KnnMatch(cv::InputArray desp1, cv::InputArray desp2, int k) {
    MatchResKnn res;
    cv::Mat mask(desp1.rows(), desp2.rows(), CV_8UC1);
    Get_BF_Matcher(this->dis_mode)->knnMatch(desp1, desp2, res, k);
    return res;
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

class LoopBody : public cv::ParallelLoopBody {
   public:
    LoopBody(const std::vector<cv::Mat>& _desp1, const std::vector<cv::Mat>& _desp2, std::vector<std::vector<cv::DMatch>>& res_, int k)
        : desp1(_desp1), desp2(_desp2), res(res_), k(k) {}

    virtual void operator()(const cv::Range& r) const {
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
    }

   protected:
    const std::vector<cv::Mat>& desp2;
    const std::vector<cv::Mat>& desp1;
    std::vector<std::vector<cv::DMatch>>& res;
    int k = 0;
};
MatchResKnn Matcher::KnnMatch_(const std::vector<cv::Mat>& desp1, const std::vector<cv::Mat>& desp2, int k, uint n_threads) {
    assert(desp2.size() >= k && desp1.size());
    MatchResKnn ret;
    ret.resize(desp1.size(), std::vector<cv::DMatch>(k));
    LoopBody body(desp1, desp2, ret, 2);
    cv::parallel_for_(cv::Range(0, static_cast<int>(desp1.size())), body);  //启动
    return ret;
}
}  // namespace MCVSLAM