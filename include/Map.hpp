#ifndef MAP_H
#define MAP_H
#include <deque>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "Pinhole.hpp"

#pragma once
#include "BaseCamera.hpp"
#include "osg_viewer.hpp"

namespace MCVSLAM {
class MapPoint;
class Frame;
class Object;
class Map;

using FrameRef = Frame *;
using KeyFrame = FrameRef;
using MapPointRef = std::shared_ptr<MapPoint>;
using ObjectRef = std::shared_ptr<Object>;

class Map {
   public:
    Map(std::string config_file);
    ~Map(){};
    static uint cnt_kf, used_kf, cnt_mp, used_mp;
    MapPointRef CreateMappoint(double x, double y, double z, cv::Mat _desp, uint kf_id, CAM_NAME cam_name);
    MapPointRef CreateMappoint(cv::Mat xyz, cv::Mat _desp, uint kf_id, CAM_NAME cam_name);
    FrameRef CreateFrame(cv::Mat imgleft, cv::Mat imgright, cv::Mat imgwide, double time_stamp, BaseCamera *cam_left, BaseCamera *cam_right,
                         BaseCamera *cam_wide);

    void AddKeyFrame(FrameRef frame);
    void DelKeyFrame(FrameRef frame);
    void DelAllMappointObservation(MapPointRef mp);

    void Clear();
    // Mapping
    int TrangularizationTwoObject(ObjectRef obj1, ObjectRef obj2, KeyFrame kf1, KeyFrame kf2, float min_baseline,
                                  const std::vector<float> &uright_obj1 = {}, const std::vector<float> &uright_obj2 = {});

    static cv::Mat ComputeF12(ObjectRef &rig1, ObjectRef &rig2);

    // statistics
    std::vector<cv::Mat> GetAllMappointsForShow(CAM_NAME cam_name);
    std::vector<cv::Mat> GetAllKeyFrameForShow();
    int MapPointSize();
    int KeyFrameSize();

    // keyframe gragh
    void UpdateConnections(KeyFrame kf);

    void AddLoopConnection(KeyFrame kf0, KeyFrame kf1);
    void DelLoopConnection(KeyFrame kf0, KeyFrame kf1);

    // grab some subgraph by some strategies
    std::unordered_set<KeyFrame> GrabLocalMap_EssGraph(KeyFrame kf, uint hop);
    // grab by frame's relative mappoint, used when this frame has not updated connection.
    std::unordered_set<KeyFrame> GrabLocalMap_Mappoint(FrameRef frame, uint hop);

    std::unordered_set<MapPointRef> GrabLocalMappoint(const std::unordered_set<KeyFrame> &local_kfs, CAM_NAME name);

    std::unordered_set<KeyFrame> GrabAnchorKeyFrames(std::unordered_set<KeyFrame> &kfs);

    // Tracjtory
    void AddFramePose(cv::Mat Tcw, KeyFrame rkf);

   public:
    std::unordered_set<MapPointRef> all_mappoints;
    std::unordered_set<KeyFrame> all_keyframes;

    std::vector<cv::Mat> frame_wise_pose_relative_kf;

    uint mp_id = 0;
    uint frame_id = 0;

    // keyframe gragh
    boost::shared_mutex mtx_connection;
    std::unordered_map<KeyFrame, std::unordered_set<KeyFrame>> essential_gragh;
    std::unordered_map<KeyFrame, KeyFrame> parent_kf;
    std::unordered_map<KeyFrame, std::unordered_set<KeyFrame>> loop_kfs;

    // configure variables
    uint connection_threshold;
    uint mappoint_life_span;

    // recent_created_mappoints , need to check if those mappoing is ok.
    std::deque<MapPointRef> recent_created_mps;

    // Tracjtory    [tracking ]
    std::vector<std::pair<cv::Mat, KeyFrame>> trajectories;
};

}  // namespace MCVSLAM
#endif