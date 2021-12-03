#include "Object.hpp"

#include <cstddef>
#include <opencv2/core/types.hpp>
#include <type_traits>

#include "Map.hpp"
#include "Matcher.hpp"
#include "Vocabulary.h"
using namespace std;

namespace MCVSLAM {

DBoW3::Vocabulary Object::voc;

Object::~Object() {}

Object::Object(MCVSLAM::BaseCamera *_cam, cv::Mat _img, ORB *_extractor, CAM_NAME name) : mpCam(_cam), img(_img), name(name), extractor(_extractor) {
    SetPose(cv::Mat::eye(4, 4, CV_32F));
    bounddingbox = cv::Rect(0, 0, img.cols, img.rows);
    grid_width_inv = static_cast<double>(FRAME_GRID_COLS) / img.cols;
    grid_width_inv = static_cast<double>(FRAME_GRID_ROWS) / img.rows;
}

void Object::SetPose(cv::Mat Tcw) {
    WRITELOCK lock(mtx_pose);
    // std::unique_lock<std::mutex> lock(mtxPose);
    mTcw = Tcw.clone();
    UpdatePoseMatrix();
}

cv::Mat Object::GetPose() {
    READLOCK lock(mtx_pose);
    // std::unique_lock<std::mutex> lock(mtxPose);
    return mTcw.clone();
}

cv::Mat Object::GetPoseInverse() {
    READLOCK lock(mtx_pose);
    // std::unique_lock<std::mutex> lock(mtxPose);
    return mTwc.clone();
}

cv::Mat Object::GetCameraCenter() {
    READLOCK lock(mtx_pose);
    // std::unique_lock<std::mutex> lock(mtxPose);
    return mOw.clone();
}

cv::Mat Object::GetRotation() {
    READLOCK lock(mtx_pose);
    // std::unique_lock<std::mutex> lock(mtxPose);
    return mRcw.clone();
}

cv::Mat Object::GetTranslation() {
    READLOCK lock(mtx_pose);
    // std::unique_lock<std::mutex> lock(mtxPose);
    return mtcw.clone();
}

std::vector<MapPointRef> Object::GetMapPointsVector() {
    std::vector<MapPointRef> ret;
    {
        UNIQUELOCK lock(mtx_mps);
        ret.assign(all_mps.begin(), all_mps.end());
    }
    return ret;
}

std::unordered_set<MapPointRef> Object::GetMapPoints() {
    UNIQUELOCK lock(mtx_mps);
    return all_mps;
}

void Object::clear() {
    UNIQUELOCK lock(mtx_mps);
    mMP2IDX.clear();
    mIDX2MP.clear();
    all_mps.clear();
}

bool Object::count(MapPointRef pMP) {
    if (pMP == NULL) return false;
    UNIQUELOCK lock(mtx_mps);
    if (mMP2IDX.count(pMP) == 0) return false;
    return true;
}

bool Object::count(size_t idx) {
    if (idx < 0 || idx >= size()) return false;
    UNIQUELOCK lock(mtx_mps);
    if (mIDX2MP.count(idx) == 0) return false;
    return true;
}

size_t Object::GetMapPointIdx(MapPointRef pMP) {
    if (pMP == NULL) {
        fmt::print("NULL pMP \n");
        while (1)
            ;
        return -1;
    }
    UNIQUELOCK lock(mtx_mps);
    //	data_check();
    if (mMP2IDX.count(pMP) == 0) {
        fmt::print("Not exist \n");
        while (1)
            ;
        return -1;
    }

    size_t idx = mMP2IDX[pMP];
    if (mIDX2MP.count(idx) == 0 || mIDX2MP[idx] != pMP) {
        fmt::print("Info not correct \n");
        while (1)
            ;
        return -1;
    }

    return idx;
}

MapPointRef Object::GetMapPoint(size_t idx) {
    UNIQUELOCK lock(mtx_mps);
    if (idx >= size() || mIDX2MP.count(idx) == 0) return NULL;
    //	data_check();

    MapPointRef pMP = mIDX2MP[idx];
    if (mMP2IDX.count(pMP) == 0 || mMP2IDX[pMP] != idx) return NULL;
    return pMP;
}

void Object::replaceMapPoint(MapPointRef pMP, MapPointRef pMP1) {
    if (pMP == NULL || pMP1 == NULL) return;
    if (count(pMP) == false) return;
    UNIQUELOCK lock(mtx_mps);
    size_t idx_ori = mMP2IDX[pMP];
    mIDX2MP.erase(idx_ori);
    mMP2IDX.erase(pMP);
    all_mps.erase(pMP);

    mIDX2MP[idx_ori] = pMP1;
    mMP2IDX[pMP1] = idx_ori;
    all_mps.insert(pMP1);
}

void Object::DelMapPoint(size_t idx) {
    if (idx >= size()) return;
    UNIQUELOCK lock(mtx_mps);
    // std::unique_lock<std::mutex> lock(mtxMapPoints);
    if (mIDX2MP.count(idx) == 0) return;
    MapPointRef pMP = mIDX2MP[idx];
    if (mIDX2MP.count(idx)) mIDX2MP.erase(idx);
    if (mMP2IDX.count(pMP)) mMP2IDX.erase(pMP);
    if (all_mps.count(pMP)) all_mps.erase(pMP);
}

void Object::DelMapPoint(MapPointRef pMP) {
    if (pMP == NULL) return;
    UNIQUELOCK lock(mtx_mps);
    if (mMP2IDX.count(pMP) == 0) return;
    size_t idx = mMP2IDX[pMP];
    if (mIDX2MP.count(idx)) mIDX2MP.erase(idx);
    if (mMP2IDX.count(pMP)) mMP2IDX.erase(pMP);
    if (all_mps.count(pMP)) all_mps.erase(pMP);
}

size_t Object::MapPointSize() {
    UNIQUELOCK lock(mtx_mps);
    return all_mps.size();
}

uint Object::Covisibility(const ObjectRef &other) {
    UNIQUELOCK lock(mtx_mps);
    auto mps1 = other->GetMapPoints();
    uint cnt = 0;
    for (const auto mp : all_mps) {
        if (mps1.count(mp)) {
            cnt++;
        }
    }
    return cnt;
}

/*
    解决设置mappoint 冲突的策略
    strategy 1： 直接使用新的地图点覆盖掉.
    strategy 1： 自动比对两个地图点的ORB描述子的距离，择优设置.
 */

// #define MAPPOINT_STRATEGY_COMPARE
#define MAPPOINT_STRATEGY_FORCE

void Object::AddMapPoint(MapPointRef pMP, size_t idx) {
    DelMapPoint(pMP);
    DelMapPoint(idx);

    {
        UNIQUELOCK _lock(mtx_mps);
        // if (mIDX2MP[idx]) {
        //     // check distance
        //     uint dist_ori = HammingDistance(pMP->GetDesp(), desps.row(idx));
        //     uint dist_new = HammingDistance(mIDX2MP[idx]->GetDesp(), desps.row(idx));
        //     if (dist_new < dist_ori) {
        //         // double delete for force guaranty
        //         DelMapPoint(pMP);
        //     } else {
        //         // do nothing
        //         return;
        //     }
        // }
        mIDX2MP[idx] = pMP;
        mMP2IDX[pMP] = idx;
        all_mps.insert(pMP);
    }
}

float Object::ComputeSceneMedianDepth() {
    cv::Mat Tcw_ = GetPose();
    vector<float> vDepths;
    vDepths.reserve(size());
    cv::Mat Rwc = GetRotation().t();
    float zcw = Tcw_.at<float>(2, 3);

    // int cnt_mines_depth = 0;
    for (MapPointRef mp : GetMapPoints()) {
        // there still statistic the outlier's depth;
        cv::Mat x3Dw = mp->GetWorldPos();
        float z = Rwc.dot(x3Dw) + zcw;
        vDepths.push_back(z);
    }
    sort(vDepths.begin(), vDepths.end());
    return vDepths[(vDepths.size()) / 2];
}

void Object::UpdatePoseMatrix() {
    mRcw = mTcw.rowRange(0, 3).colRange(0, 3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0, 3).col(3);
    mOw = -mRwc * mtcw;

    mTwc = cv::Mat::eye(4, 4, mTcw.type());
    mRwc.copyTo(mTwc.rowRange(0, 3).colRange(0, 3));
    mOw.copyTo(mTwc.rowRange(0, 3).col(3));
}

void Object::AssignFeaturesToGrid() {
    // Fill matrix with points
    const int nCells = FRAME_GRID_COLS * FRAME_GRID_ROWS;

    int nReserve = 0.5f * size() / (nCells);

    for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
        for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++) {
            grid[i][j].reserve(nReserve);
        }

    for (int i = 0; i < size(); i++) {
        const cv::KeyPoint &kp = kps[i];

        int nGridPosX, nGridPosY;
        if (PosInGrid(kp, nGridPosX, nGridPosY)) {
            grid[nGridPosX][nGridPosY].push_back(i);
        }
    }
}

uint Object::ProjectBunchMapPoints(const std::vector<MapPointRef> &mps, float r_threshold) {
    std::unordered_set<MapPointRef> maps_set(mps.begin(), mps.end());
    return ProjectBunchMapPoints(maps_set, r_threshold);
}

uint Object::ProjectBunchMapPoints(const std::unordered_set<MapPointRef> &mps, float r_threshold) {
    uint cnt = 0;
    for (const MapPointRef &mp : mps) {
        cv::Point2f pt = this->mpCam->project(this->GetRotation() * mp->GetWorldPos() + this->GetTranslation());

        std::vector<cv::Mat> _desps;

        for (size_t idx : GetFeaturesInArea(pt.x, pt.y, r_threshold)) {
            _desps.push_back(desps.row(idx));
        }
        if (_desps.size()) {
            MatchRes res = Matcher::KnnMatch({mp->GetDesp()}, _desps).FilterRatio(0.8);
            if (res.size()) {
                AddMapPoint(mp, res[0].trainIdx);
                cnt += 1;
            }
        }
    }
    return cnt;
}

void Object::ComputeBow() {
    if (is_bowed == false) {
        std::vector<cv::Mat> v_desps;
        for (int i = 0, sz = desps.rows; i < sz; i++) {
            v_desps.push_back(desps.row(i));
        }
        voc.transform(v_desps, bow_vector, bow_feature, 4);
        is_bowed = true;
    }
}

bool Object::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY) {
    posX = round((kp.pt.x - bounddingbox.tl().x) * grid_width_inv);
    posY = round((kp.pt.y - bounddingbox.tl().y) * grid_height_inv);

    // Keypoint's coordinates are undistorted, which could cause to go out of the
    // image
    if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS) return false;
    return true;
}

vector<size_t> Object::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel) const {
    vector<size_t> vIndices;
    vIndices.reserve(size());

    float factorX = r;
    float factorY = r;

    const int nMinCellX = max(0, (int)floor((x - bounddingbox.tl().x - factorX) * grid_width_inv));
    if (nMinCellX >= FRAME_GRID_COLS) return vIndices;
    const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - bounddingbox.tl().x + factorX) * grid_width_inv));
    if (nMaxCellX < 0) return vIndices;

    const int nMinCellY = max(0, (int)floor((y - bounddingbox.tl().y - factorY) * grid_height_inv));
    if (nMinCellY >= FRAME_GRID_ROWS) return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - bounddingbox.tl().y + factorY) * grid_height_inv));
    if (nMaxCellY < 0) return vIndices;

    const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

    for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
        for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
            const vector<size_t> vCell = grid[ix][iy];
            if (vCell.empty()) continue;
            // cout << "\t " << ix << " " << iy << endl;
            for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
                // cout << vCell[j] << " ";
                const cv::KeyPoint &kp = kps[vCell[j]];
                if (bCheckLevels) {
                    if (kp.octave < minLevel) continue;
                    if (maxLevel >= 0)
                        if (kp.octave > maxLevel) continue;
                }

                const float distx = kp.pt.x - x;
                const float disty = kp.pt.y - y;

                if (fabs(distx) < factorX && fabs(disty) < factorY) vIndices.push_back(vCell[j]);
            }
            // cout << endl;
        }
    }
    return vIndices;
}

}  // namespace MCVSLAM