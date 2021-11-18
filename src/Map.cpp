#include "Map.hpp"

#include <memory>

#include "MapPoint.hpp"
namespace MCVSLAM {
MapPointRef Map::CreateMappoint(double x, double y, double z, cv::Mat _desp) {
    MapPoint* map = new MapPoint(x, y, z, _desp, mp_id++);
    // MapPointRef ret = std::make_shared<MapPoint>(x, y, z, _desp);
    MapPointRef ret = std::shared_ptr<MapPoint>(map);
    all_mappoints.push_back(ret);
    return ret;
}

int Map::Size() { return all_mappoints.size(); }

}  // namespace MCVSLAM