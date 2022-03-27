#include "Tracker.hpp"

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <memory>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

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

Tracker::Tracker(Map *_map, osg_viewer *_viewer, const std::string &config_file) : map(_map), viewer(_viewer) {
    Yaml::Node root;
    Yaml::Parse(root, config_file);
    Th_depth = root["Th_depth"].As<float>();
    Th_motionmodel_min_mps = root["Th_motionmodel_min_mps"].As<float>();
    Th_local_map_min_mps = root["Th_local_map_min_mps"].As<float>();
    Th_lastkeyframe_min_mps = root["Th_lastkeyframe_min_mps"].As<float>();
    Th_max_frame_interval = root["Th_max_frame_interval"].As<uint>();
}

Tracker::~Tracker() {}

Track_State Tracker::Track(cv::Mat imgleft, cv::Mat imgright, cv::Mat imgwide, double time_stamp, BaseCamera *cam_left, BaseCamera *cam_right,
                           BaseCamera *cam_wide) {
    FrameRef cur_frame = map->CreateFrame(imgleft, imgright, imgwide, time_stamp, cam_left, cam_left, cam_wide, GetLastKeyFrame(3));

    Track_State state = Track_State::OK;

    if (GetLastKeyFrame() == nullptr) {
        uint init_cnt_mps = Init(cur_frame);
        if (init_cnt_mps > 80) {
            // init success
            map->AddKeyFrame(cur_frame);
            SetLastKeyFrame(cur_frame);
            SetLastFrame(cur_frame);
            // {
            //     cv::Mat T_cur_rfk = cur_frame->GetPoseInverse() * GetLastKeyFrame()->GetPose();
            //     map->AddFramePose(T_cur_rfk, GetLastKeyFrame(), cur_frame->time_stamp);
            // }
            state = Track_State::OK;
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
                MyTimer::Timer _("TLastK");

                uint cnt_l = Bow_Track(GetLastKeyFrame()->LEFT, cur_frame->LEFT);
                uint cnt_w = Bow_Track(GetLastKeyFrame()->WIDE, cur_frame->WIDE);

                cur_frame->SetPose(GetLastFrame()->GetPose());
                int opcnt = PoseEstimation::PoseOptimization(cur_frame);
                // fmt::print("[track LastKeyFrame] left: {} wide:{}, oped {}\n", cnt_l, cnt_w, opcnt);
                if (opcnt < 50) {
                    fmt::print("Track LastKeyFrame Faild! \n");
                    // exit(0);
                    state = Track_State::LOST;
                }
            } else if (!velocity.empty()) {
                // track by MotionModel
                MyTimer::Timer _("TMotion");
                cur_frame->SetPose(velocity * GetLastFrame()->GetPose());
                if (viewer) {
                    viewer->SetCurrentCamera(cur_frame->GetPose());
                }

                auto lkfmps = GetLastKeyFrame()->LEFT->GetMapPoints();
                auto lkfmps_wide = GetLastKeyFrame()->WIDE->GetMapPoints();
                uint pcnt = cur_frame->LEFT->ProjectBunchMapPoints(lkfmps, 5);
                uint pcntw = cur_frame->WIDE->ProjectBunchMapPoints(lkfmps_wide, 5);

                int cnt = PoseEstimation::PoseOptimization(cur_frame);
                _.tock();
                // auto k = getchar();

                if (viewer) {
                    viewer->SetCurrentCamera(cur_frame->GetPose());
                }
                // fmt::print("track MotionModel {} project left {} project wide  {}\n", cnt, pcnt, pcntw);
                if (cnt < Th_motionmodel_min_mps) {
                    fmt::print("Track MotionModel Faild! \n");
                    state = Track_State::TRACK_FAILD;
                    if (GetLastKeyFrame()) {
                        // track by bow with LastKeyFrame
                        uint cnt_l = Bow_Track(GetLastKeyFrame()->LEFT, cur_frame->LEFT);
                        uint cnt_w = Bow_Track(GetLastKeyFrame()->WIDE, cur_frame->WIDE);
                        cur_frame->SetPose(GetLastFrame()->GetPose());
                        int opcnt = PoseEstimation::PoseOptimization(cur_frame);
                        // fmt::print("[track LastKeyFrame] left: {} wide:{}, oped {}\n", cnt_l, cnt_w, opcnt);
                        if (opcnt < 20) {
                            fmt::print("Track LastKeyFrame Faild! \n");
                            // exit(0);
                            state = Track_State::TRACK_FAILD;
                        } else {
                            state = Track_State::OK;
                        }
                    }
                }
            }

            // if (state == Track_State::OK)
            {
                // track local map
                MyTimer::Timer _("TLocal");
                auto local_kfs = map->GrabLocalMap_Mappoint(cur_frame, 3);
                auto local_mps = map->GrabLocalMappoint(local_kfs, CAM_NAME::L);
                auto local_mps_wide = map->GrabLocalMappoint(local_kfs, CAM_NAME::W);
                uint pcnt = cur_frame->LEFT->ProjectBunchMapPoints(local_mps, 10);
                uint pcnt_w = cur_frame->WIDE->ProjectBunchMapPoints(local_mps_wide, 7);
                int opcnt = PoseEstimation::PoseOptimization(cur_frame);

                // fmt::print("[track local map]{}  {} kfs, {} mps,  project left {} wide {}\n", opcnt, local_kfs.size(), local_mps.size(), pcnt,
                //            pcnt_w);
                if (opcnt < 50) {
                    fmt::print("Track local map Faild! \n");
                    // exit(0);
                    state = Track_State::LOST;
                } else {
                    state = Track_State::OK;
                }
            }
        }
        if (state == Track_State::LOST) {
            // delete cur_frame;
            // this->Clear();
            // map->Clear();
            fmt::print("Track LOST with {} frame Please Press [q] key to exit\n", cur_frame->id);
            // while (getchar() != 'q')
            //     ;
            if (viewer) viewer->RequestStop();
            return Track_State::STOP;
        }

        // Calculate velocity
        if (GetLastFrame()) {
            velocity = cur_frame->GetPose() * GetLastFrame()->GetPoseInverse();
        }

        // Check if need new keyframe
        if (CheckNeedNewKeyFrame(cur_frame)) {
            if (viewer) {
                viewer->Draw(map->GetAllMappointsForShow(CAM_NAME::L), 225, 0, 0);
                viewer->Draw(map->GetAllMappointsForShow(CAM_NAME::W), 0, 225, 2);
                viewer->DrawPredictTracjectories(map->GetAllKeyFrameForShow(), map->GetAllKeyFrameMaskForShow(), 0, 0, 225);
                viewer->SetCurrentCamera(cur_frame->GetPose());
                viewer->DrawEssentialGraph(map->GetEssentialGraph());
                viewer->Commit();
            }
            // getchar();
            map->AddKeyFrame(cur_frame);
            SetLastKeyFrame(cur_frame);
        }
    }

    SetLastFrame(cur_frame);
    // fmt::print("kf {}/{} mp {}/{} \n", Map::cnt_kf, Map::used_kf, Map::cnt_mp,
    // Map::used_mp); fmt::print(" {} {} kfs in map  {} {} mps in map \n",
    // Map::cnt_kf - Map::used_kf, map->KeyFrameSize(), Map::cnt_mp -
    // Map::used_mp,
    //            map->MapPointSize());

    // Commit this frame to the Trajectory
    {
        cv::Mat T_cur_rfk = cur_frame->GetPose() * GetLastKeyFrame()->GetPoseInverse();
        map->AddFramePose(T_cur_rfk, GetLastKeyFrame(), cur_frame->time_stamp, GetLastKeyFrame() == cur_frame);
    }
    if (viewer) {
        // fmt::print("asdfasdfads");
        viewer->Draw(map->GetAllMappointsForShow(CAM_NAME::L), 225, 0, 0);
        viewer->Draw(map->GetAllMappointsForShow(CAM_NAME::W), 0, 225, 2);
        viewer->DrawPredictTracjectories(map->GetAllKeyFrameForShow(), map->GetAllKeyFrameMaskForShow(), 0, 0, 225);
        viewer->SetCurrentCamera(cur_frame->GetPose());
        viewer->DrawEssentialGraph(map->GetEssentialGraph());
        viewer->Commit();
    }

    // char _ = getchar();
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

std::vector<KeyFrame> Tracker::GetLastKeyFrame(uint cnt) {
    std::vector<KeyFrame> kfs;
    if (queue_keyframe.empty()) return kfs;
    for (auto rit = queue_frame.rbegin(), rend = queue_frame.rend(); cnt && rit != rend; rit++) {
        kfs.push_back(*rit);
    }
    return kfs;
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

bool Tracker::CheckNeedNewKeyFrame(KeyFrame cur_frame) {
    // Tracked MapPoints in the reference keyframe
    KeyFrame lastkf = GetLastKeyFrame();
    if (lastkf == nullptr) return true;

    int min_obs = 3;
    uint obs_f_kf = cur_frame->LEFT->Covisibility(lastkf->LEFT);
    uint mps_kf = lastkf->LEFT->MapPointSize();
    // Check how many "close" points are being tracked and how many could be
    // potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose = 0;
    std::vector<float> &depth_left = cur_frame->depth_left;
    for (int i = 0, sz = depth_left.size(); i < sz; i++) {
        float z = depth_left[i];
        if (z > 0 && z < Th_depth) {
            if (cur_frame->LEFT->GetMapPoint(i))
                nTrackedClose++;
            else
                nNonTrackedClose++;
        }
    }

    bool bNeedToInsertClose;
    bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

    // Thresholds
    float thRefRatio = 0.75f;
    // Condition 1a: More than "MaxFrames" have passed from last keyframe
    // insertion
    const bool c1a = cur_frame->id >= GetLastKeyFrame()->id + Th_max_frame_interval;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle

    // const bool c1b = ((cur_frame->id >= mnLastKeyFrameId + mMinFrames) &&
    // bLocalMappingIdle);

    // Condition 1c: tracking is weak
    const bool c1c = (obs_f_kf < mps_kf * 0.25 || bNeedToInsertClose);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of
    // visual odometry compared to map matches.
    const bool c2 = (((obs_f_kf < mps_kf * thRefRatio || bNeedToInsertClose)) && obs_f_kf > 15);

    // Temporal condition for Inertial cases

    if (((c1a || c1c) && c2)) {
        return true;
    }
    return false;
}

uint Tracker::Init(KeyFrame &cur_frame) {
    // stereo init
    // create mappoints / not initialized
    uint cnt = 0;
    for (uint i = 0, sz = cur_frame->LEFT->size(); i < sz; i++) {
        cv::KeyPoint kp = cur_frame->LEFT->kps[i];
        float depth = cur_frame->depth_left[i];
        if (depth != -1) {
            cv::Mat pos = cur_frame->LEFT->mpCam->unproject_z(kp.pt, depth);
            // cout << mp << endl;
            auto mpr =
                map->CreateMappoint(pos, cur_frame->LEFT->desps.row(i), cur_frame->LEFT->kps[i].octave, cur_frame->id, CAM_NAME::L, MP_TYPE::STEREO);
            cur_frame->LEFT->AddMapPoint(mpr, i);
            mpr->ProjectResRecord(true);
            cnt += 1;
        }
    }
    return cnt;
}

uint Tracker::Bow_Track(ObjectRef obj1, ObjectRef obj2) {
    obj1->ComputeBow();
    obj2->ComputeBow();
    MatchRes res = Matcher::DBowMatch(obj1->desps, obj1->bow_feature, obj2->desps, obj2->bow_feature)
                       .FilterRatio()
                       .FilterThreshold()
                       .FilterOrientation(obj1->kps, obj2->kps);
    cv::Mat show_img;
    // cv::drawMatches(obj1->img, obj1->kps, obj2->img, obj2->kps, res, show_img);
    // cv::imshow("show_img", show_img);
    // cv::waitKey(10);
    uint cnt = 0;
    for (const auto &p : res) {
        MapPointRef mp = obj1->GetMapPoint(p.queryIdx);
        if (mp != NULL) {
            obj2->AddMapPoint(mp, p.trainIdx);
            cnt++;
        }
    }
    return cnt;
}

uint Tracker::Wnd_Track(ObjectRef obj1, ObjectRef obj2) {
    uint cnt = 0;
    MatchRes allres;
    for (uint idx : obj1->GetAllMapPointsIdxs()) {
        cv::KeyPoint kp = obj1->kps[idx];
        cv::Mat desp = obj1->desps.row(idx);
        auto candi_idxs = obj2->GetFeaturesInArea(kp.pt, 20);
        std::vector<cv::Mat> desps;
        std::vector<cv::KeyPoint> kps;
        for (uint idx_target : candi_idxs) {
            desps.push_back(obj2->desps.row(idx_target));
            kps.push_back(obj2->kps[idx_target]);
        }
        MatchRes t_res = Matcher::KnnMatch({desp}, desps).FilterRatio().FilterThreshold().FilterOrientation({kp}, kps);
        if (t_res.size()) {
            allres.push_back(cv::DMatch(idx, candi_idxs[t_res[0].queryIdx], t_res[0].distance));
            obj2->AddMapPoint(obj1->GetMapPoint(idx), candi_idxs[t_res[0].queryIdx]);
            cnt++;
        }
    }
    // if (allres.size()) {
    //     cv::Mat outimg;
    //     cv::drawMatches(obj1->img, obj1->kps, obj2->img, obj2->kps, allres, outimg);
    //     fmt::print("{}\n", cnt);
    //     cv::imshow("img", outimg);
    //     cv::waitKey(0);
    // }
    return cnt;
}

}  // namespace MCVSLAM