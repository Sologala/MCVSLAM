#include "MapPoint.hpp"
namespace MCVSLAM {
MapPoint::MapPoint(double x, double y, double z, cv::Mat _desp, uint _id): 
id(_id) {
    position_w = cv::Mat({x, y, z});
    desp = _desp;
}

MapPoint::~MapPoint() {}
}  // namespace MCVSLAM