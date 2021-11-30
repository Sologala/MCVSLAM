#include "Tracker.hpp"

#include <iostream>
#include <opencv2/video/tracking.hpp>

#include "Frame.hpp"
#include "Map.hpp"
#include "MapPoint.hpp"
#include "Matcher.hpp"
#include "Object.hpp"
#include "PoseEstimation.hpp"
#include "pyp/timer/timer.hpp"
#include "pyp/yaml/yaml.hpp"
using namespace std;
namespace MCVSLAM {

Tracker::Tracker(Map* _map, osg_viewer* _viewer) : map(_map), viewer(_viewer) {}

Tracker::~Tracker() {}

void Tracker::Track(FrameRef cur_frame) {
    if (last_frame == nullptr) {
        // create mappoints / not initialized
        for (uint i = 0, sz = cur_frame->LEFT->size(); i < sz; i++) {
            cv::KeyPoint kp = cur_frame->LEFT->kps[i];
            float depth = cur_frame->depth_left[i];
            if (depth != -1) {
                cv::Mat mp = cur_frame->LEFT->mpCam->unproject_z(kp.pt, depth);
                // cout << mp << endl;
                auto mpr = map->CreateMappoint(mp, cur_frame->LEFT->desps.row(i));
                cur_frame->LEFT->AddMapPoint(mpr, i);
            }
        }
        // create keyframes

        map->AddKeyFrame(cur_frame);

        last_keyframe = cur_frame;
    } else {
        {
            // track by motion model + last keyframe
            if (velocity.empty() == false) {
                cur_frame->SetPose(last_frame->GetPose() * velocity);
                cur_frame->LEFT->ProjectBunchMapPoints(last_keyframe->LEFT->GetMapPoints());
                cur_frame->WIDE->ProjectBunchMapPoints(last_keyframe->WIDE->GetMapPoints());
                PoseEstimation::PoseOptimization(cur_frame);
            }
        }

        // {
        //     // Track last Frame
        //     ObjectRef f1 = last_frame->LEFT, f2 = cur_frame->LEFT;
        //     MatchRes match_res = Matcher::KnnMatch_cv(f1->desps, f2->desps).FilterRatio(0.6);
        //     cv::Mat T = PoseEstimation::_2d2d(f1, f2, match_res);
        //     cur_frame->SetPose(last_frame->GetPose() * T);
        // }

        {
            // track local map
            std::unordered_set<KeyFrame> localkfs = map->GrabSubGraph_Mappoint(cur_frame, 2);
            std::unordered_set<MapPointRef> local_maps = map->GrabLocalMappoint(localkfs);
            cur_frame->LEFT->ProjectBunchMapPoints(local_maps);
            PoseEstimation::PoseOptimization(cur_frame);
            // get local map
        }
        cout << cur_frame->GetPose() << endl;
    }
    viewer->Draw(map->GetAllMappointsForShow(), 0, 0, 225);
    viewer->Commit();
    last_frame = cur_frame;
    fmt::print("{:^20}{:^20}{:^20}\n", "item", "time(ms)", "fps");
    for (auto p : MyTimer::Timer::COUNT) {
        fmt::print("{:^20}{:^20.6f}{:^20.6f}\n", p.first, p.second.ms(), p.second.fps());
    }
}

void Tracker::KL_Track(ObjectRef obj1, Object obj2) {
    // cv::Mat res, err;
    // cv::calcOpticalFlowPyrLK(obj1->img, obj2.img, obj1->kps, obj2.kps, res, err);
}

}  // namespace MCVSLAM