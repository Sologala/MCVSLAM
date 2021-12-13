#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <boost/thread.hpp>
#include <boost/thread/pthread/mutex.hpp>
#include <memory>
#include <opencv2/core/types.hpp>
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
    virtual ~Object();
    Object(MCVSLAM::BaseCamera *_cam, cv::Mat _img, ORB *_extractor, CAM_NAME name);

    // ---------------------[Statistic] ----------------------
    size_t size() const { return kps.size(); }

    size_t MapPointSize();

    // Covisibility
    uint Covisibility(const ObjectRef &other);

    // ---------------------[Pose] ----------------------
    // Set the camera pose. (Imu pose is not modified!)

    void SetPose(cv::Mat Tcw) {
        WRITELOCK lock(mtx_pose);
        // std::unique_lock<std::mutex> lock(mtxPose);
        mTcw = Tcw.clone();
        UpdatePoseMatrix();
    }

    cv::Mat GetPose() {
        READLOCK lock(mtx_pose);
        // std::unique_lock<std::mutex> lock(mtxPose);
        return mTcw.clone();
    }
    cv::Mat GetPoseInverse() {
        READLOCK lock(mtx_pose);
        // std::unique_lock<std::mutex> lock(mtxPose);
        return mTwc.clone();
    }

    cv::Mat GetCameraCenter() {
        READLOCK lock(mtx_pose);
        // std::unique_lock<std::mutex> lock(mtxPose);
        return mOw.clone();
    }
    cv::Mat GetRotation() {
        READLOCK lock(mtx_pose);
        // std::unique_lock<std::mutex> lock(mtxPose);
        return mRcw.clone();
    }

    cv::Mat GetTranslation() {
        READLOCK lock(mtx_pose);
        // std::unique_lock<std::mutex> lock(mtxPose);
        return mtcw.clone();
    }

    cv::Mat Map(const cv::Mat &x3D) {
        READLOCK lock(mtx_pose);
        return mRcw * x3D + mtcw;
    }
    cv::Mat Map(const MapPointRef &mpr) { return Map(mpr->GetWorldPos()); }

    cv::Point2f Project(const cv::Mat &x3D) { return mpCam->project(x3D); }

    // ------------------------[Map Points]--------------------

    bool count(MapPointRef pMP) {
        if (pMP == NULL) return false;
        READLOCK lock(mtx_mps);
        if (mMP2IDX.count(pMP) == 0) return false;
        return true;
    }
    bool count(size_t idx) {
        if (idx < 0 || idx >= size()) return false;
        READLOCK lock(mtx_mps);
        if (mIDX2MP.count(idx) == 0) return false;
        return true;
    }

    std::vector<MapPointRef> GetMapPointsVector() {
        std::vector<MapPointRef> ret;
        {
            READLOCK lock(mtx_mps);
            ret.assign(all_mps.begin(), all_mps.end());
        }
        return ret;
    }

    std::unordered_set<MapPointRef> GetMapPoints() {
        READLOCK lock(mtx_mps);
        return all_mps;
    }
    std::unordered_set<MapPointRef> GetMapPoints(uint th_obs) {
        READLOCK lock(mtx_mps);
        std::unordered_set<MapPointRef> ret;
        for (auto &mp : all_mps) {
            if (mp->GetObservationCnt() >= th_obs) ret.insert(mp);
        }
        return ret;
    }
    std::unordered_set<uint> GetAllMapPointsIdxs() {
        READLOCK lock(mtx_mps);
        std::unordered_set<uint> ret;
        for (const auto &p : mIDX2MP) {
            ret.insert(p.first);
        }
        return ret;
    }
    size_t GetMapPointIdx(MapPointRef pMP);

    MapPointRef GetMapPoint(size_t idx);

    void AddMapPoint(MapPointRef pMP, size_t idx);

    void ReplaceMapPoint(MapPointRef mp1, MapPointRef mp2);

    void DelMapPoint(size_t idx);

    void DelMapPoint(MapPointRef pMP);

    // mappoint normal & median depth
    float ComputeSceneMedianDepth();

    // ---------------------[Grid] ----------------------
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    std::vector<size_t> GetFeaturesInArea(const cv::Point2f uv, const float &r, const int minLevel = -1, const int maxLevel = -1) const;
    std::vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel = -1, const int maxLevel = -1) const;

    // ---------------------[Camera parameter]
    float GetParameter(const CAM_PARA i) { return mpCam->getParameter(i); }

    // ------------------[repenish some information]---------------------------
    void repenish(std::vector<MapPointRef> &vMaps, std::vector<cv::KeyPoint> &vKPs, std::vector<cv::Mat> &vDesps);

    void AssignFeaturesToGrid();

    //  project a bunch of mappoints to this frame with current pose;
    uint ProjectBunchMapPoints(const std::vector<MapPointRef> &mps, float r_threshold = 5);
    uint ProjectBunchMapPoints(const std::unordered_set<MapPointRef> &mps, float r_threshold = 5);

    // Bow index
    void ComputeBow();

   protected:
    // ---------------------[Pose] ----------------------
    void UpdatePoseMatrix();

   protected:
    boost::shared_mutex mtx_mps;
    boost::shared_mutex mtx_pose;  // for pose

   public:
    CAM_NAME name;
    BaseCamera *mpCam;

    cv::Mat img;
    cv::Rect bounddingbox;

    Keypoints kps;
    Desps desps;

    // Bow index
    bool is_bowed = false;
    DBoW3::BowVector bow_vector;
    DBoW3::FeatureVector bow_feature;

    static DBoW3::Vocabulary voc;
    ORB *extractor;

    void clear() {
        WRITELOCK lock(mtx_mps);
        mMP2IDX.clear();
        mIDX2MP.clear();
        all_mps.clear();
    }

   private:
    Grid grid;
    float grid_width_inv;
    float grid_height_inv;

    // MapPoint observation
    std::unordered_map<MapPointRef, size_t> mMP2IDX;
    std::unordered_map<size_t, MapPointRef> mIDX2MP;
    std::unordered_set<MapPointRef> all_mps;

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
