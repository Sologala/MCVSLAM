#ifndef MAPPOINT_H
#define MAPPOINT_H
#pragma once
#include <opencv2/core/mat.hpp>
#include <unordered_map>
#include <vector>
namespace MCVSLAM {
class Map;

class MapPoint {
    friend class Map;

   public:
    ~MapPoint();
    cv::Mat GetWorldPos() { return position_w; }

   public:
    // Position in absolute coordinates
    cv::Mat position_w;
    cv::Mat desp;
    uint id;

   private:
    MapPoint(double x, double y, double z, cv::Mat _desp, uint id);
};

using MapPointRef = std::shared_ptr<MapPoint>;

}  // namespace MCVSLAM
#endif