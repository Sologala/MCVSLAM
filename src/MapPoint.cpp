#include "MapPoint.hpp"

#include <opencv2/core/mat.hpp>

#include "Frame.hpp"
#include "Object.hpp"
using namespace std;
namespace MCVSLAM {
MapPoint::MapPoint(double x, double y, double z, cv::Mat _desp, uint _kf_id, uint _id, CAM_NAME _created_from, uint _life_span)
    : id(_id), kf_id(_kf_id), create_from(_created_from), lifespan(_life_span) {
    position_w = (cv::Mat_<float>(3, 1) << x, y, z);
    desp = _desp;
}

MapPoint::~MapPoint() {
    WRITELOCK _lock(mtx_pos);
    WRITELOCK _lock1(mtx_feature);
    Map::used_mp += 1;
}

const cv::Mat MapPoint::GetWorldPos() {
    READLOCK _lock(mtx_pos);
    return position_w.clone();
}

void MapPoint::SetWorldPose(const cv::Mat &pos) {
    WRITELOCK _lock(mtx_pos);
    position_w = pos.clone();
}

const cv::Mat MapPoint::GetDesp() {
    READLOCK _lock(mtx_feature);
    return desp;
}

void MapPoint::BindKeyFrame(KeyFrame kf, ObjectRef obj) {
    WRITELOCK _lock(mtx_feature);
    relative_kfs[kf].insert(obj);
}

void MapPoint::UnBindKeyFrame(KeyFrame kf, ObjectRef obj) {
    WRITELOCK _lock(mtx_feature);
    if (relative_kfs.count(kf)) {
        if (relative_kfs[kf].count(obj)) relative_kfs[kf].erase(obj);
        if (relative_kfs[kf].size() == 0) {
            relative_kfs.erase(kf);
        }
    }
}

const std::unordered_set<KeyFrame> MapPoint::GetAllKeyFrame() {
    READLOCK _lock(mtx_feature);
    std::unordered_set<KeyFrame> ret;
    for (const auto &p : relative_kfs) {
        ret.insert(p.first);
    }
    return ret;
}

const Observation MapPoint::GetAllObservation() {
    READLOCK _lock(mtx_feature);
    return relative_kfs;
}

cv::Mat MapPoint::GetNormalVector() {
    READLOCK _lock(mtx_feature);
    return norm_vec.clone();
}

void MapPoint::ComputeDistinctiveDescriptors() {
    // Retrieve all observed descriptors
    auto obs_kf = GetAllObservation();
    if (obs_kf.empty()) return;
    vector<cv::Mat> all_ob_desps;
    all_ob_desps.reserve(obs_kf.size());

    for (auto p : obs_kf) {
        KeyFrame kf = p.first;
        // Replace measurement in keyframe
        for (const auto &obj : p.second) {
            size_t idx = obj->GetMapPointIdx(this->shared_from_this());
            all_ob_desps.push_back(obj->desps.row(idx));
        }
    }

    if (all_ob_desps.empty()) return;

    // Compute distances between them
    size_t N = all_ob_desps.size();

    // ajon
    std::vector<std::vector<int>> mpDistanceBuffer(N, std::vector<int>(N, 0));

    for (size_t i = 0; i < N; i++) {
        for (size_t j = i + 1; j < N; j++) {
            int distij = HammingDistance(all_ob_desps[i], all_ob_desps[j]);
            mpDistanceBuffer[i][j] = distij;
            mpDistanceBuffer[j][i] = distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for (size_t i = 0; i < N; i++) {
        sort(mpDistanceBuffer[i].begin(), mpDistanceBuffer[i].end());
        int median = mpDistanceBuffer[i][0.5 * (N - 1)];

        if (median < BestMedian) {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        WRITELOCK lock1(mtx_feature);
        desp = all_ob_desps[BestIdx].clone();
    }
}

bool MapPoint::isBad() {
    READLOCK _lock(mtx_feature);
    return is_bad;
}

void MapPoint::SetBad() {
    WRITELOCK _lock(mtx_feature);
    is_bad = true;
}
void MapPoint::UpdateNormalVector() {
    Observation obs = GetAllObservation();
    cv::Mat pos = GetWorldPos();

    cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
    int n = 0;
    for (const std::pair<const KeyFrame, std::unordered_set<ObjectRef>> p : obs) {
        KeyFrame kf = p.first;
        for (ObjectRef obj : p.second) {
            cv::Mat Owi = obj->GetCameraCenter();
            cv::Mat normali = pos - Owi;
            normal = normal + normali / cv::norm(normali);
            n++;
        }
    }

    {
        WRITELOCK lock(mtx_feature);
        norm_vec = normal / n;
    }
}

}  // namespace MCVSLAM