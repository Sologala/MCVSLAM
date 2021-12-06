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
    Track_State state = Track_State::OK;

    if (GetLastKeyFrame() == nullptr) {
        uint init_cnt_mps = Init(cur_frame);
        if (init_cnt_mps > 80) {
            // init success
            map->AddKeyFrame(cur_frame);
            SetLastKeyFrame(cur_frame);
            SetLastFrame(cur_frame);
            {
                cv::Mat T_cur_rfk = cur_frame->GetPoseInverse() * GetLastKeyFrame()->GetPose();
                map->AddFramePose(T_cur_rfk, GetLastKeyFrame());
            }
            return Track_State::OK;
        } else {
            // init faild
            delete cur_frame;
            map->Clear();
            Clear();
            return Track_State::INIT_FAILD;
        }

    } else {
        {
            // system has been initialized!
            if (velocity.empty() || (GetLastKeyFrame() && GetLastKeyFrame()->id + 5 >= cur_frame->id)) {
                // track by bow with LastKeyFrame
                uint cnt_l = Bow_Track(GetLastKeyFrame()->LEFT, cur_frame->LEFT);
                uint cnt_w = Bow_Track(GetLastKeyFrame()->WIDE, cur_frame->WIDE);
                cur_frame->SetPose(GetLastFrame()->GetPose());
                int opcnt = PoseEstimation::PoseOptimization(cur_frame);
                fmt::print("[track LastKeyFrame] left: {} wide:{}, oped {}\n", cnt_l, cnt_w, opcnt);
                if (opcnt < 50) {
                    fmt::print("Track LastKeyFrame Faild! \n");
                    // exit(0);
                    state = Track_State::LOST;
                }
            } else if (0 && !velocity.empty()) {
                // track by MotionModel
                cur_frame->SetPose(GetLastFrame()->GetPose() * velocity);
                uint pcnt = cur_frame->LEFT->ProjectBunchMapPoints(GetLastKeyFrame()->LEFT->GetMapPoints());
                uint pcntw = cur_frame->WIDE->ProjectBunchMapPoints(GetLastKeyFrame()->WIDE->GetMapPoints());
                int cnt = PoseEstimation::PoseOptimization(cur_frame);
                fmt::print("track MotionModel {} project left {} project wide  {}\n", cnt, pcnt, pcntw);
                if (cnt < 50) {
                    fmt::print("Track MotionModel Faild! \n");
                    state = Track_State::TRACK_FAILD;
                    if (GetLastKeyFrame()) {
                        // track by bow with LastKeyFrame
                        uint cnt_l = Bow_Track(GetLastKeyFrame()->LEFT, cur_frame->LEFT);
                        uint cnt_w = Bow_Track(GetLastKeyFrame()->WIDE, cur_frame->WIDE);
                        cur_frame->SetPose(GetLastFrame()->GetPose());
                        int opcnt = PoseEstimation::PoseOptimization(cur_frame);
                        fmt::print("[track LastKeyFrame] left: {} wide:{}, oped {}\n", cnt_l, cnt_w, opcnt);
                        if (opcnt < 20) {
                            fmt::print("Track LastKeyFrame Faild! \n");
                            // exit(0);
                            state = Track_State::TRACK_FAILD;
                        }
                    }
                }
            }

            // if (state == Track_State::OK)
            {
                // track local map
                auto local_kfs = map->GrabLocalMap_Mappoint(cur_frame, 3);
                auto local_mps = map->GrabLocalMappoint(local_kfs, CAM_NAME::L);
                auto local_mps_wide = map->GrabLocalMappoint(local_kfs, CAM_NAME::W);
                uint pcnt = cur_frame->LEFT->ProjectBunchMapPoints(local_mps, 5);
                uint pcnt_w = cur_frame->WIDE->ProjectBunchMapPoints(local_mps_wide, 5);
                int opcnt = PoseEstimation::PoseOptimization(cur_frame);
                fmt::print("[track local map]{}  {} kfs, {} mps,  project left {} wide {}\n", opcnt, local_kfs.size(), local_mps.size(), pcnt,
                           pcnt_w);
                if (opcnt < 50) {
                    fmt::print("Track local map Faild! \n");
                    // exit(0);
                    state = Track_State::LOST;
                } else {
                    state = Track_State::OK;
                }
            }
        }
    }

    if (state == Track_State::LOST) {
        // delete cur_frame;
        this->Clear();
        map->Clear();
        fmt::print("Track LOST with {} frame Please Press any key to exit\n", cur_frame->id);
        char _ = getchar();
        viewer->RequestStop();
        exit(0);
    }

    viewer->Draw(map->GetAllMappointsForShow(), 225, 0, 0);
    viewer->DrawCams(map->GetAllKeyFrameForShow(), 225, 0, 0);
    viewer->Commit();

    // Calculate velocity
    if (GetLastFrame()) {
        velocity = GetLastKeyFrame()->GetPose().inv() * cur_frame->GetPose();
    }

    // Check if need new keyframe
    if ((cur_frame->id - GetLastKeyFrame()->id > 4)) {
        map->AddKeyFrame(cur_frame);
        SetLastKeyFrame(cur_frame);
    }
    SetLastFrame(cur_frame);
    // fmt::print("kf {}/{} mp {}/{} \n", Map::cnt_kf, Map::used_kf, Map::cnt_mp, Map::used_mp);
    // fmt::print(" {} {} kfs in map  {} {} mps in map \n", Map::cnt_kf - Map::used_kf, map->KeyFrameSize(), Map::cnt_mp - Map::used_mp,
    //            map->MapPointSize());

    // Commit this frame to the Trajectory
    {
        cv::Mat T_cur_rfk = cur_frame->GetPoseInverse() * GetLastKeyFrame()->GetPose();
        map->AddFramePose(T_cur_rfk, GetLastKeyFrame());
    }
    fmt::print("{:^20}{:^20}{:^20}\n", "item", "time(ms)", "fps");
    for (auto p : MyTimer::Timer::COUNT) {
        fmt::print("{:^20}{:^20.6f}{:^20.6f}\n", p.first, p.second.ms(), p.second.fps());
    }
    return Track_State::OK;
}

void Tracker::Clear() {
    while (!queue_frame.empty()) {
        KeyFrame pframe = queue_frame.front();
        if (pframe->is_marked_no_free == false) delete pframe;
        queue_frame.pop_front();
    }

    // queue_keyframe.clear();
    fmt::print("Tracker cleared\n");
}

uint Tracker::Bow_Track(ObjectRef obj1, ObjectRef obj2) {
    obj1->ComputeBow();
    obj2->ComputeBow();
    MatchRes res = Matcher::DBowMatch(obj1->desps, obj1->bow_feature, obj2->desps, obj2->bow_feature).FilterRatio().FilterThreshold();
    cv::Mat show_img;
    // cv::drawMatches(obj1->img, obj1->kps, obj2->img, obj2->kps, res, show_img);
    // cv::imshow("show_img", show_img);
    // cv::waitKey(10);
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
    kf->is_marked_no_free = true;
}

void Tracker::SetLastFrame(FrameRef f) {
    while (queue_frame.size() >= 3) {
        FrameRef pframe = queue_frame.front();
        // the frame which not be set as keyframe will be free here.
        if (pframe->is_marked_no_free == false) delete pframe;

        queue_frame.pop_front();
    }
    queue_frame.push_back(f);
}

uint Tracker::Init(KeyFrame& cur_frame) {
    // stereo init
    // create mappoints / not initialized
    uint cnt = 0;
    for (uint i = 0, sz = cur_frame->LEFT->size(); i < sz; i++) {
        cv::KeyPoint kp = cur_frame->LEFT->kps[i];
        float depth = cur_frame->depth_left[i];
        if (depth != -1) {
            cv::Mat mp = cur_frame->LEFT->mpCam->unproject_z(kp.pt, depth);
            // cout << mp << endl;
            auto mpr = map->CreateMappoint(mp, cur_frame->LEFT->desps.row(i), cur_frame->id);
            cur_frame->LEFT->AddMapPoint(mpr, i);
            cnt += 1;
        }
    }
    return cnt;
}

void Tracker::KL_Track(ObjectRef obj1, ObjectRef obj2) {
    // cv::Mat res, err;
    // cv::calcOpticalFlowPyrLK(obj1->img, obj2.img, obj1->kps, obj2.kps, res, err);
}

}  // namespace MCVSLAM