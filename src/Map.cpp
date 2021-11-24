#include "Map.hpp"

#include <memory>

#include "MapPoint.hpp"
namespace MCVSLAM {
MapPointRef Map::CreateMappoint(double x, double y, double z, cv::Mat _desp) {
    MapPoint* mp = new MapPoint(x, y, z, _desp, mp_id++);
    // MapPointRef ret = std::make_shared<MapPoint>(x, y, z, _desp);
    MapPointRef ret = std::shared_ptr<MapPoint>(mp);
    all_mappoints.push_back(ret);
    return ret;
}

MapPointRef Map::CreateMappoint(cv::Mat xyz, cv::Mat _desp) {
    assert(!xyz.empty() && xyz.rows == 3);
    return CreateMappoint(xyz.at<float>(0), xyz.at<float>(1), xyz.at<float>(2), _desp);
}

void Map::AddKeyFrame(FrameRef frame) { all_keyframes.push_back(frame); }

std::vector<cv::Mat> Map::GetAllMappointsForShow() {
    std::vector<cv::Mat> ret;
    for (const MapPointRef& mpr : all_mappoints) {
        ret.push_back(mpr->position_w);
    }
    return ret;
}

int Map::Size() { return all_mappoints.size(); }

}  // namespace MCVSLAM