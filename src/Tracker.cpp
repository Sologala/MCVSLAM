#include "Tracker.hpp"

#include <algorithm>
#include <iostream>
#include <memory>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
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

Track_State Tracker::Track(FrameRef cur_frame) {
    if (GetLastKeyFrame() == nullptr) {
        // create mappoints / not initialized
        for (uint i = 0, sz = cur_frame->LEFT->size(); i < sz; i++) {
            cv::KeyPoint kp = cur_frame->LEFT->kps[i];
            float depth = cur_frame->depth_left[i];
            if (depth != -1) {
                cv::Mat mp = cur_frame->LEFT->mpCam->unproject_z(kp.pt, depth);
                // cout << mp << endl;
                auto mpr = map->CreateMappoint(mp, cur_frame->LEFT->desps.row(i), cur_frame->id);
                cur_frame->LEFT->AddMapPoint(mpr, i);
            }
        }
        // create keyframes

        map->AddKeyFrame(cur_frame);
        SetLastKeyFrame(cur_frame);
    } else {
        {
            if (velocity.empty() == false) {
                // track by MotionModel

                cur_frame->SetPose(GetLastFrame()->GetPose() * velocity);
                uint pcnt = cur_frame->LEFT->ProjectBunchMapPoints(GetLastKeyFrame()->LEFT->GetMapPoints());
                uint pcntw = cur_frame->WIDE->ProjectBunchMapPoints(GetLastKeyFrame()->WIDE->GetMapPoints());
                int cnt = PoseEstimation::PoseOptimization(cur_frame);
                fmt::print("track MotionModel {} project left {} project wide  {}\n", cnt, pcnt, pcntw);
                if (cnt == 0) {
                    fmt::print("Track MotionModel Faild! \n");
                    return Track_State::NORMAL;
                }
            } else {
                // track by bow with LastKeyFrame
                uint cnt_l = Bow_Track(GetLastKeyFrame()->LEFT, cur_frame->LEFT);
                uint cnt_w = Bow_Track(GetLastKeyFrame()->WIDE, cur_frame->WIDE);
                int opcnt = PoseEstimation::PoseOptimization(cur_frame);
                fmt::print("track LastKeyFrame left: {} wide:{}, oped {}\n", cnt_l, cnt_w, opcnt);
                if (opcnt == 0) {
                    fmt::print("Track LastKeyFrame Faild! \n");
                    // exit(0);
                    this->Clear();
                    delete cur_frame;
                    map->Clear();
                    return Track_State::LOST;
                }
            }

            {
                // track local map
                auto local_kfs = map->GrabLocalMap_Mappoint(cur_frame, 3);
                auto local_mps = map->GrabLocalMappoint(local_kfs, CAM_NAME::L);
                uint pcnt = cur_frame->LEFT->ProjectBunchMapPoints(local_mps);
                int opcnt = PoseEstimation::PoseOptimization(cur_frame);
                fmt::print("track local map {} kfs, {} mps,  project left {}\n", local_kfs.size(), local_mps.size(), pcnt);
                if (opcnt == 0) {
                    fmt::print("Track local map Faild! \n");
                    fmt::print("{}\n", GetLastKeyFrame() == nullptr);
                    // exit(0);
                }
            }
        }
        // cout << cur_frame->GetPose() << endl;
        SetLastFrame(cur_frame);
    }
    viewer->Draw(map->GetAllMappointsForShow(), 225, 0, 0);
    viewer->Commit();

    // Check Mem Leaky

    // fmt::print("kf {}/{} mp {}/{} \n", Map::cnt_kf, Map::used_kf, Map::cnt_mp, Map::used_mp);
    // fmt::print(" {} {} kfs in map  {} {} mps in map \n", Map::cnt_kf - Map::used_kf, map->KeyFrameSize(), Map::cnt_mp - Map::used_mp,
    //            map->MapPointSize());

    fmt::print("{:^20}{:^20}{:^20}\n", "item", "time(ms)", "fps");
    for (auto p : MyTimer::Timer::COUNT) {
        fmt::print("{:^20}{:^20.6f}{:^20.6f}\n", p.first, p.second.ms(), p.second.fps());
    }
    return Track_State::NORMAL;
}

void Tracker::Clear() {
    while (!queue_frame.empty()) {
        delete queue_frame.front();
        queue_frame.pop_front();
    }

    queue_keyframe.clear();
}

uint Tracker::Bow_Track(ObjectRef obj1, ObjectRef obj2) {
    obj1->ComputeBow();
    obj2->ComputeBow();
    MatchRes res = Matcher::DBowMatch(obj1->desps, obj1->bow_feature, obj2->desps, obj2->bow_feature).FilterRatio().FilterThreshold();
    uint cnt = 0;
    for (const auto& p : res) {
        MapPointRef mp = obj1->GetMapPoint(p.queryIdx);
        if (mp != NULL) {
            obj2->AddMapPoint(mp, p.trainIdx);
            cnt++;
        }
    }
    return cnt;
}

KeyFrame Tracker::GetLastKeyFrame() {
    if (queue_keyframe.empty()) return nullptr;
    return queue_keyframe.back();
}

FrameRef Tracker::GetLastFrame() {
    if (queue_frame.empty()) return nullptr;
    return queue_frame.back();
}

void Tracker::SetLastKeyFrame(KeyFrame kf) {
    while (queue_keyframe.size() >= 3) queue_keyframe.pop_front();
    queue_keyframe.push_back(kf);
}

void Tracker::SetLastFrame(FrameRef f) {
    while (queue_frame.size() >= 3) {
        FrameRef pframe = queue_frame.front();
        // the frame which not be set as keyframe will be free here.
        delete pframe;

        queue_frame.pop_front();
    }
    queue_frame.push_back(f);
}

void Tracker::KL_Track(ObjectRef obj1, ObjectRef obj2) {
    // cv::Mat res, err;
    // cv::calcOpticalFlowPyrLK(obj1->img, obj2.img, obj1->kps, obj2.kps, res, err);
}

}  // namespace MCVSLAM