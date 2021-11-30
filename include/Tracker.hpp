#ifndef TRACKER_H
#define TRACKER_H
#include <string>

#include "Map.hpp"
#include "Pinhole.hpp"
#include "osg_viewer.hpp"

#pragma once
namespace MCVSLAM {

class Tracker {
   public:
    Tracker(Map* _map, osg_viewer* _viewer);
    ~Tracker();

    void Track(FrameRef frame);

    void KL_Track(ObjectRef obj1, Object obj2);

    Map* map;
    osg_viewer* viewer;
    float baseline = 0.8;
    float bf = 764.324024;
    FrameRef last_frame;
    KeyFrame last_keyframe;

    cv::Mat velocity;
};

}  // namespace MCVSLAM
#endif