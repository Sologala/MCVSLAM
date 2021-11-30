#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <boost/thread.hpp>
#include <boost/thread/pthread/mutex.hpp>
#include <memory>
#include <unordered_set>
#include <vector>

#include "BaseCamera.hpp"
#include "BowVector.h"
#include "DBoW3.h"
#include "FeatureVector.h"
#include "MapPoint.hpp"
#include "Matcher.hpp"
#include "ORBExtractor.hpp"
#include "Pinhole.hpp"
#include "Vocabulary.h"
#include "pyp/fmt/fmt.hpp"
using READLOCK = boost::shared_lock<boost::shared_mutex>;
using WRITELOCK = boost::unique_lock<boost::shared_mutex>;
using UNIQUELOCK = boost::unique_lock<boost::mutex>;
#pragma once
namespace MCVSLAM {

static unsigned int FRAME_GRID_ROWS = 30;
static unsigned int FRAME_GRID_COLS = 30;
class Grid : public std::vector<std::vector<std::vector<std::size_t>>> {
   public:
    Grid() { resize(FRAME_GRID_COLS, std::vector<std::vector<std::size_t>>(FRAME_GRID_ROWS)); }
};

class Object {
   public:
    virtual ~Object(){};
    Object(MCVSLAM::BaseCamera *_cam, cv::Mat _img, ORB *_extractor, CAM_NAME name);

    // ---------------------[Statistic] ----------------------
    size_t size() const { return kps.size(); }

    // ---------------------[Pose] ----------------------
    // Set the camera pose. (Imu pose is not modified!)

    void SetPose(cv::Mat Tcw) {
        WRITELOCK lock(mtxPose);
        // std::unique_lock<std::mutex> lock(mtxPose);
        mTcw = Tcw.clone();
        UpdatePoseMatrix();
    }

    cv::Mat GetPose() {
        READLOCK lock(mtxPose);
        // std::unique_lock<std::mutex> lock(mtxPose);
        return mTcw.clone();
    }
    cv::Mat GetPoseInverse() {
        READLOCK lock(mtxPose);
        // std::unique_lock<std::mutex> lock(mtxPose);
        return mTwc.clone();
    }
    cv::Mat GetCameraCenter() {
        READLOCK lock(mtxPose);
        // std::unique_lock<std::mutex> lock(mtxPose);
        return mOw.clone();
    }
    cv::Mat GetRotation() {
        READLOCK lock(mtxPose);
        // std::unique_lock<std::mutex> lock(mtxPose);
        return mRcw.clone();
    }
    cv::Mat GetTranslation() {
        READLOCK lock(mtxPose);
        // std::unique_lock<std::mutex> lock(mtxPose);
        return mtcw.clone();
    }
    // ------------------------[Map Points]--------------------

    std::vector<MapPointRef> GetMapPoints() {
        std::vector<MapPointRef> ret;
        {
            UNIQUELOCK lock(mtxMapPoints);
            for (const std::pair<MapPointRef, size_t> &p : mMP2IDX) {
                ret.push_back(p.first);
            }
        }
        return ret;
    }

    void clear() {
        UNIQUELOCK lock(mtxMapPoints);
        mMP2IDX.clear();
        mIDX2MP.clear();
    }

    bool count(MapPointRef pMP) {
        if (pMP == NULL) return false;
        UNIQUELOCK lock(mtxMapPoints);
        if (mMP2IDX.count(pMP) == 0) return false;
        return true;
    }
    bool count(size_t idx) {
        if (idx < 0 || idx >= size()) return false;
        UNIQUELOCK lock(mtxMapPoints);
        if (mIDX2MP.count(idx) == 0) return false;
        return true;
    }

    size_t GetMapPointIdx(MapPointRef pMP) {
        if (pMP == NULL) {
            fmt::print("NULL pMP \n");
            while (1)
                ;
            return -1;
        }
        UNIQUELOCK lock(mtxMapPoints);
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

    MapPointRef GetMapPoint(size_t idx) {
        UNIQUELOCK lock(mtxMapPoints);
        if (idx >= size() || mIDX2MP.count(idx) == 0) return NULL;
        //	data_check();

        MapPointRef pMP = mIDX2MP[idx];
        if (mMP2IDX.count(pMP) == 0 || mMP2IDX[pMP] != idx) return NULL;
        return pMP;
    }

    void AddMapPoint(MapPointRef pMP, size_t idx) {
        if (idx >= size() || pMP == NULL) return;
        {
            UNIQUELOCK lock(mtxMapPoints);
            if (mMP2IDX.count(pMP) || mIDX2MP.count(idx)) {
                // check distance
                uint dist_ori = HammingDistance(pMP->GetDesp(), desps.row(idx));
                uint dist_new = HammingDistance(mIDX2MP[idx]->GetDesp(), desps.row(idx));
                if (dist_new < dist_ori) {
                    // double delete for force guaranty
                    DelMapPoint(pMP);
                    DelMapPoint(idx);
                } else {
                    // do nothing
                    return;
                }
            }
            mIDX2MP[idx] = pMP;
            mMP2IDX[pMP] = idx;
        }
    }

    void replaceMapPoint(MapPointRef pMP, MapPointRef pMP1) {
        if (pMP == NULL || pMP1 == NULL) return;
        if (count(pMP) == false) return;
        UNIQUELOCK lock(mtxMapPoints);
        size_t idx_ori = mMP2IDX[pMP];
        mIDX2MP.erase(idx_ori);
        mMP2IDX.erase(pMP);

        mIDX2MP[idx_ori] = pMP1;
        mMP2IDX[pMP1] = idx_ori;
        //	data_check();
    }

    void DelMapPoint(size_t idx) {
        if (idx >= size()) return;
        UNIQUELOCK lock(mtxMapPoints);
        // std::unique_lock<std::mutex> lock(mtxMapPoints);
        if (mIDX2MP.count(idx) == 0) return;
        MapPointRef pMP = mIDX2MP[idx];
        mIDX2MP.erase(idx);
        mMP2IDX.erase(pMP);
    }

    void DelMapPoint(MapPointRef pMP) {
        if (pMP == NULL) return;
        UNIQUELOCK lock(mtxMapPoints);
        if (mMP2IDX.count(pMP) == 0) return;
        size_t idx = mMP2IDX[pMP];
        mIDX2MP.erase(idx);
        mMP2IDX.erase(pMP);
    }
    size_t MapPointSize() {
        UNIQUELOCK lock(mtxMapPoints);
        size_t ret = mMP2IDX.size();
        return ret;
    }

    // mappoint normal & median depth
    float ComputeSceneMedianDepth();

    // ---------------------[Grid] ----------------------
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    std::vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel = -1, const int maxLevel = -1) const;

    // ---------------------[Camera parameter]
    float GetParameter(const CAM_PARA i) { return mpCam->getParameter(i); }

    // ------------------[repenish some information]---------------------------
    void repenish(std::vector<MapPointRef> &vMaps, std::vector<cv::KeyPoint> &vKPs, std::vector<cv::Mat> &vDesps);

    void AssignFeaturesToGrid();

    //  project a bunch of mappoints to this frame with current pose;
    void ProjectBunchMapPoints(const std::vector<MapPointRef> &mps, float r_threshold = 5);
    void ProjectBunchMapPoints(const std::unordered_set<MapPointRef> &mps, float r_threshold = 5);

    // Bow index
    void ComputeBow();

   protected:
    // ---------------------[Pose] ----------------------
    void UpdatePoseMatrix();

   protected:
    boost::mutex mtxMapPoints;
    boost::shared_mutex mtxPose;  // for pose

   public:
    CAM_NAME name;
    BaseCamera *mpCam;
    cv::Mat img;

    cv::Rect bounddingbox;

    Keypoints kps;
    Desps desps;

    // Bow index
    DBoW3::BowVector bow_vector;
    DBoW3::FeatureVector bow_feature;

    // Sparse storage;
    std::vector<size_t> idxs;
    std::vector<MapPointRef> MPs;
    //  for recored the outliers MapPoints.
    std::unordered_set<MapPointRef> mOutliers;

    static DBoW3::Vocabulary voc;
    ORB *extractor;

   private:
    Grid grid;
    float grid_width_inv;
    float grid_height_inv;

    std::unordered_map<MapPointRef, size_t> mMP2IDX;
    std::unordered_map<size_t, MapPointRef> mIDX2MP;

    // ------------[cam parameter]------------
    // Calibration parameters

    // -------------[POse]--------------
    cv::Mat mRwc;
    cv::Mat mOw;
    cv::Mat mTcw;
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mTwc;
};

}  // namespace MCVSLAM

#endif  // OBJECT_HPP
