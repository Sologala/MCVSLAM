#ifndef MAPPOINT_H
#define MAPPOINT_H
#include <boost/thread/pthread/mutex.hpp>
#include <boost/thread/pthread/shared_mutex.hpp>
#include <memory>

#include "Pinhole.hpp"
#pragma once
#include <boost/thread.hpp>
#include <opencv2/core/mat.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Map.hpp"
namespace MCVSLAM {

using Observation = std::unordered_map<KeyFrame, std::unordered_set<ObjectRef>>;

class MapPoint : public std::enable_shared_from_this<MapPoint> {
    friend class Map;

   public:
    MapPoint(double x, double y, double z, cv::Mat _desp, uint _level, uint _kf_id, uint _id, CAM_NAME _created_from, uint _life_span);

    ~MapPoint();

    const cv::Mat GetWorldPos();
    void SetWorldPose(const cv::Mat& pos);

    const cv::Mat GetDesp();
    const uint GetID() const { return id; };

    void BindKeyFrame(KeyFrame kf, ObjectRef obj);
    void UnBindKeyFrame(KeyFrame kf, ObjectRef obj);

    const std::unordered_set<KeyFrame> GetAllKeyFrame();
    const Observation GetAllObservation();
    const uint GetObservationCnt();
    void UpdateConnection();
    // normal adn median depth
    cv::Mat GetNormalVector();
    void UpdateNormalVector();
    void ComputeDistinctiveDescriptors();

    bool isBad();
    void SetBad();

    // found
    void ProjectResRecord(bool _project_res);
    float GetProjectSucessRate();

   public:
    // Position in absolute coordinates
    cv::Mat position_w;
    // historical position
    std::vector<cv::Mat> historic_position_w;
    cv::Mat desp;
    uint level;
    uint id;
    uint kf_id;
    boost::shared_mutex mtx_pos;

    boost::shared_mutex mtx_feature;

    const CAM_NAME create_from;
    bool is_bad = false;
    cv::Mat norm_vec;
    Observation relative_kfs;

    // [found_cnt] statistics the number of times when success to project this mappoint to a frame and find a match in it .
    uint project_cnt = 0;
    uint project_sucess_cnt = 0;

    // Mappoint LiftSpan
    uint lifespan;
};

}  // namespace MCVSLAM
#endif