#include "MapPoint.hpp"

#include <opencv2/core/mat.hpp>
namespace MCVSLAM {
MapPoint::MapPoint(double x, double y, double z, cv::Mat _desp, uint _id) : id(_id) {
    position_w = (cv::Mat_<float>(3, 1) << x, y, z);
    desp = _desp;
}

MapPoint::~MapPoint() {}
}  // namespace MCVSLAM