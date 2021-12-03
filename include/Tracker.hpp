#ifndef TRACKER_H
#define TRACKER_H
#include <stack>
#include <string>

#include "Map.hpp"
#include "Pinhole.hpp"
#include "osg_viewer.hpp"

#pragma once
namespace MCVSLAM {
enum Track_State { LOST = 1, NORMAL = 2 };
class Tracker {
   public:
    Tracker(Map* _map, osg_viewer* _viewer);
    ~Tracker();

    Track_State Track(FrameRef frame);
    void Clear();
    void KL_Track(ObjectRef obj1, ObjectRef obj2);
    uint Bow_Track(ObjectRef obj1, ObjectRef obj2);

    KeyFrame GetLastKeyFrame();
    FrameRef GetLastFrame();
    void SetLastKeyFrame(KeyFrame kf);
    void SetLastFrame(FrameRef kf);

    Map* map;
    osg_viewer* viewer;
    float baseline = 0.8;
    float bf = 764.324024;

    std::deque<FrameRef> queue_frame;
    std::deque<KeyFrame> queue_keyframe;
    cv::Mat velocity;
};

}  // namespace MCVSLAM
#endif