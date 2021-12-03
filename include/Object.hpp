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
    virtual ~Object();
    Object(MCVSLAM::BaseCamera *_cam, cv::Mat _img, ORB *_extractor, CAM_NAME name);

    // ---------------------[Statistic] ----------------------
    size_t size() const { return kps.size(); }

    size_t MapPointSize();

    // Covisibility
    uint Covisibility(const ObjectRef &other);

    // ---------------------[Pose] ----------------------
    // Set the camera pose. (Imu pose is not modified!)

    void SetPose(cv::Mat Tcw);

    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // ------------------------[Map Points]--------------------
    void clear();

    bool count(MapPointRef pMP);
    bool count(size_t idx);

    std::vector<MapPointRef> GetMapPointsVector();
    std::unordered_set<MapPointRef> GetMapPoints();

    size_t GetMapPointIdx(MapPointRef pMP);

    MapPointRef GetMapPoint(size_t idx);

    void AddMapPoint(MapPointRef pMP, size_t idx);

    void replaceMapPoint(MapPointRef pMP, MapPointRef pMP1);

    void DelMapPoint(size_t idx);

    void DelMapPoint(MapPointRef pMP);

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
    uint ProjectBunchMapPoints(const std::vector<MapPointRef> &mps, float r_threshold = 5);
    uint ProjectBunchMapPoints(const std::unordered_set<MapPointRef> &mps, float r_threshold = 5);

    // Bow index
    void ComputeBow();

   protected:
    // ---------------------[Pose] ----------------------
    void UpdatePoseMatrix();

   protected:
    boost::mutex mtx_mps;
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
