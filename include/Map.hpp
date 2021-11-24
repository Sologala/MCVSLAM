#ifndef MAP_H
#define MAP_H
#include <memory>
#include <unordered_map>
#pragma once

#include "Frame.hpp"
#include "MapPoint.hpp"
#include "osg_viewer.hpp"
namespace MCVSLAM {
class Map {
   public:
    Map(){};
    ~Map(){};

    MapPointRef CreateMappoint(double x, double y, double z, cv::Mat _desp);
    MapPointRef CreateMappoint(cv::Mat xyz, cv::Mat _desp);
    void AddKeyFrame(FrameRef frame);

    std::vector<cv::Mat> GetAllMappointsForShow();
    int Size();

   public:
    std::vector<MapPointRef> all_mappoints;
    std::vector<KeyFrame> all_keyframes;
    uint mp_id = 0;
};
}  // namespace MCVSLAM
#endif