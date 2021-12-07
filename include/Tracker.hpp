#ifndef TRACKER_H
#define TRACKER_H
#include <stack>
#include <string>

#include "Map.hpp"
#include "Pinhole.hpp"
#include "osg_viewer.hpp"

#pragma once
namespace MCVSLAM {
enum Track_State {
    OK = 0,
    LOST = 1,
    INIT_FAILD = 3,
    TRACK_FAILD = 2,
};
class Tracker {
   public:
    Tracker(Map* _map, osg_viewer* _viewer, const std::string& config_file);
    ~Tracker();

    Track_State Track(FrameRef frame);
    void Clear();
    void KL_Track(ObjectRef obj1, ObjectRef obj2);
    uint Bow_Track(ObjectRef obj1, ObjectRef obj2);

    KeyFrame GetLastKeyFrame();
    FrameRef GetLastFrame();

    void SetLastKeyFrame(KeyFrame kf);
    void SetLastFrame(FrameRef kf);

    bool CheckNeedNewKeyFrame(KeyFrame cur_frame);
    uint Init(KeyFrame& cur_frame);
    Map* map;
    osg_viewer* viewer;
    float baseline = 0.8;
    float bf = 764.324024;

    std::deque<FrameRef> queue_frame;
    std::deque<KeyFrame> queue_keyframe;
    cv::Mat velocity;

    // Some threshold vars, loaded from config_file
    float Th_depth;
    float Th_motionmodel_min_mps;
    float Th_local_map_min_mps;
    float Th_lastkeyframe_min_mps;
    uint Th_max_frame_interval;
};

}  // namespace MCVSLAM
#endif