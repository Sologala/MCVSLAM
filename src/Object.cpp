#include "Object.hpp"

#include <pyp/fmt/core.h>

#include <boost/thread/lock_types.hpp>
#include <cstddef>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <stdexcept>
#include <type_traits>
#include <unordered_set>

#include "Map.hpp"
#include "Matcher.hpp"
#include "ORBVocabulary.h"
using namespace std;

namespace MCVSLAM {

DBoW2::ORBVocabulary Object::voc;

Object::~Object() {}

Object::Object(MCVSLAM::BaseCamera *_cam, cv::Mat _img, ORB *_extractor, CAM_NAME name) : mpCam(_cam), img(_img), name(name), extractor(_extractor) {
    SetPose(cv::Mat::eye(4, 4, CV_32F));
    bounddingbox = cv::Rect(0, 0, img.cols, img.rows);
    grid_width_inv = static_cast<double>(FRAME_GRID_COLS) / img.cols;
    grid_height_inv = static_cast<double>(FRAME_GRID_ROWS) / img.rows;
}

size_t Object::GetMapPointIdx(MapPointRef pMP) {
    if (pMP == NULL) {
        // fmt::print("NULL pMP \n");
        return -1;
    }
    READLOCK lock(mtx_mps);
    //	data_check();
    if (mMP2IDX.count(pMP) == 0) {
        // cout << all_mps.count(pMP) << endl;
        // throw ObjectException("Not exist");
        return -1;
    }

    size_t idx = mMP2IDX[pMP];
    if (mIDX2MP.count(idx) == 0 || mIDX2MP[idx] != pMP) {
        // throw ObjectException("Info not correct ");
        return -1;
    }
    return idx;
}

MapPointRef Object::GetMapPoint(size_t idx) {
    if (idx >= size()) return NULL;
    //	data_check();
    READLOCK lock(mtx_mps);
    if (mIDX2MP.count(idx) == 0) {
        // throw std::runtime_error("Not exist");

        return NULL;
    }

    MapPointRef pMP = mIDX2MP[idx];
    if (mMP2IDX[pMP] != idx) {
        cout << mMP2IDX[pMP] << endl;
        // throw ObjectException("Info not correct ");
        return NULL;
    }
    return pMP;
}

void Object::DelMapPoint(size_t idx) {
    if (idx >= size()) return;
    WRITELOCK lock(mtx_mps);
    if (mIDX2MP.count(idx) == 0) return;
    MapPointRef pMP = mIDX2MP[idx];
    mIDX2MP.erase(idx);
    mMP2IDX.erase(pMP);
    all_mps.erase(pMP);
}

void Object::DelMapPoint(MapPointRef pMP) {
    if (pMP == NULL) return;
    WRITELOCK lock(mtx_mps);
    if (mMP2IDX.count(pMP) == 0) return;
    size_t idx = mMP2IDX[pMP];
    mIDX2MP.erase(idx);
    mMP2IDX.erase(pMP);
    all_mps.erase(pMP);
}

size_t Object::MapPointSize() {
    READLOCK lock(mtx_mps);
    return all_mps.size();
}

uint Object::Covisibility(const ObjectRef &other) {
    READLOCK lock(mtx_mps);
    auto mps1 = other->GetMapPoints();
    uint cnt = 0;
    for (const auto &mp : all_mps) {
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
    if (pMP == nullptr) return;
    DelMapPoint(pMP);
    DelMapPoint(idx);

    {
        WRITELOCK _lock(mtx_mps);
        // if (mIDX2MP[idx]) {
        //     // check distance
        //     uint dist_ori = HammingDistance(pMP->GetDesp(), desps.row(idx));
        //     uint dist_new = HammingDistance(mIDX2MP[idx]->GetDesp(),
        //     desps.row(idx)); if (dist_new < dist_ori) {
        //         // double delete for force guaranty
        //         DelMapPoint(pMP);
        //     } else {
        //         // do nothing
        //         return;
        //     }
        // }
        if (mIDX2MP.count(idx) == 0 && mMP2IDX.count(pMP) == 0 && all_mps.count(pMP) == 0) {
            mIDX2MP[idx] = pMP;
            mMP2IDX[pMP] = idx;
            all_mps.insert(pMP);
        } else {
            fmt::print("Error!!!\n");
            throw std::runtime_error("Error");
        }
    }
}

void Object::ReplaceMapPoint(MapPointRef mp1, MapPointRef mp2) {
    if (mp1->id == mp2->id) return;
    if (count(mp1) == false) return;
    uint idx = GetMapPointIdx(mp1);
    if (idx == -1) return;
    DelMapPoint(mp1);
    AddMapPoint(mp2, idx);
}

float Object::ComputeSceneMedianDepth() {
    vector<float> vDepths;
    vDepths.reserve(size());
    float zcw = GetTranslation().at<float>(2);
    cv::Mat R_row2 = GetRotation().row(2).t();
    // int cnt_mines_depth = 0;
    for (MapPointRef mp : GetMapPoints()) {
        // there still statistic the outlier's depth;
        cv::Mat x3Dw = mp->GetWorldPos();
        try {
            float z = R_row2.dot(x3Dw) + zcw;
            vDepths.push_back(z);
        } catch (cv::Exception &e) {
            cout << R_row2 << endl;
            cout << x3Dw << endl;
        }
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
        cv::Mat Pc = this->GetRotation() * mp->GetWorldPos() + this->GetTranslation();
        if (Pc.at<float>(2) < 0) continue;
        cv::Point2f pt = this->mpCam->project(Pc);

        std::vector<cv::Mat> _desps;
        std::vector<cv::KeyPoint> _kps;
        float r = r_threshold * extractor->mvScaleFactor[mp->level];
        std::vector<size_t> _ori_idx = GetFeaturesInArea(pt.x, pt.y, r);
        for (size_t idx : _ori_idx) {
            _desps.push_back(desps.row(idx));
            _kps.emplace_back(kps[idx]);
        }
        bool project_res = false;

        if (_desps.size()) {
            MatchRes res = Matcher::KnnMatch({mp->GetDesp()}, _desps).FilterRatio().FilterThreshold();
            if (res.size()) {
                AddMapPoint(mp, _ori_idx[res[0].trainIdx]);
                project_res = true;
                cnt += 1;
            }
        }
        mp->ProjectResRecord(project_res);
    }
    return cnt;
}

void Object::ComputeBow() {
    std::vector<cv::Mat> v_desps;
    WRITELOCK lock(mtx_mps);
    if (is_bowed == false) {
        for (int i = 0, sz = desps.rows; i < sz; i++) {
            cv::Mat temp;
            desps.row(i).copyTo(temp);
            v_desps.push_back(temp);
        }

        voc.transform(v_desps, bow_vector, bow_feature, 4);
        is_bowed = true;
    }

    // Check transform results
    for (auto it : bow_feature) {
        for (auto idx : it.second) {
            assert(!(idx < 0 || idx >= desps.rows));
        }
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

std::vector<size_t> Object::GetFeaturesInArea(const cv::Point2f uv, const float &r, const int minLevel, const int maxLevel) const {
    return GetFeaturesInArea(uv.x, uv.y, r, minLevel, maxLevel);
}

vector<size_t> Object::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel) const {
    vector<size_t> vIndices;
    vIndices.reserve(size());
    if (x < 0 || y < 0 || x >= bounddingbox.width || y >= bounddingbox.height) {
        return vIndices;
    }
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