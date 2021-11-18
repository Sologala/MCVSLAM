#ifndef MAP_H
#define MAP_H
#include <memory>
#include <unordered_map>
#pragma once
#include "MapPoint.hpp"
namespace MCVSLAM {
class Map {
   public:
    Map();
    ~Map();

    MapPointRef CreateMappoint(double x, double y, double z, cv::Mat _desp);
    int Size();

   public:
    std::vector<MapPointRef> all_mappoints;
    uint mp_id = 0;
};
}  // namespace MCVSLAM
#endif