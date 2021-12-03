#include "Map.hpp"

#include <algorithm>
#include <exception>
#include <memory>
#include <opencv2/core/types.hpp>
#include <unordered_set>

#include "BAoptimizer.hpp"
#include "BaseCamera.hpp"
#include "Converter.h"
#include "Frame.hpp"
#include "MapPoint.hpp"
#include "Object.hpp"
#include "Pinhole.hpp"
#include "PoseEstimation.hpp"
using namespace std;
namespace MCVSLAM {
uint Map::cnt_kf = 0, Map::used_kf = 0, Map::cnt_mp = 0, Map::used_mp = 0;
Map::Map(std::string config_file) {
    Yaml::Node root;
    Yaml::Parse(root, config_file);
    connection_threshold = root["connection_threshold"].As<int>();
}

MapPointRef Map::CreateMappoint(double x, double y, double z, cv::Mat _desp, uint kf_id) {
    // MapPoint* mp = new MapPoint(x, y, z, _desp, mp_id++);
    // MapPointRef ret = std::shared_ptr<MapPoint>(mp);
    cnt_mp += 1;
    return make_shared<MapPoint>(x, y, z, _desp, kf_id, mp_id++);
}

MapPointRef Map::CreateMappoint(cv::Mat xyz, cv::Mat _desp, uint kf_id) {
    assert(!xyz.empty() && xyz.rows == 3);
    MapPointRef ref = CreateMappoint(xyz.at<float>(0), xyz.at<float>(1), xyz.at<float>(2), _desp, kf_id);
    recent_created_mps.push_back(ref);
    return ref;
}

FrameRef Map::CreateFrame(cv::Mat imgleft, cv::Mat imgright, cv::Mat imgwide, double time_stamp, BaseCamera* cam_left, BaseCamera* cam_right,
                          BaseCamera* cam_wide) {
    cnt_kf += 1;
    return new Frame(imgleft, imgright, imgwide, time_stamp, cam_left, cam_right, cam_wide, frame_id++);
}

void Map::AddKeyFrame(FrameRef frame) {
    // recored as keyframe
    all_keyframes.insert(frame);

    //  create index from mappoints to this frame ;
    for (auto obj : {frame->LEFT, frame->WIDE, frame->RIGHT})
        for (auto mpr : obj->GetMapPoints()) {
            mpr->BindKeyFrame(frame, obj);
            AddMapPoint(mpr);
        }

    UpdateConnections(frame);

    frame->ComputeBow();
    // grab local map
    auto local_kfs = GrabLocalMap_EssGraph(frame, 2);

    // create new mappoint
    if (local_kfs.count(frame)) local_kfs.erase(frame);
    for (auto kf : local_kfs) {
        uint tri_left = TrangularizationTwoObject(kf->LEFT, frame->LEFT, kf, frame, kf->b, kf->u_right, frame->u_right);
        uint tri_wide = TrangularizationTwoObject(kf->WIDE, frame->WIDE, kf, frame, kf->b);
        fmt::print("Triangulized left: {}, wide {} \n", tri_left, tri_wide);
    }

    // culling mappoint
    for (uint i = 0, sz = recent_created_mps.size(); i < sz; i++) {
        MapPointRef mp = recent_created_mps.front();
        recent_created_mps.pop_front();
        if (mp->isBad() || frame->id - mp->kf_id >= 3 || mp->GetAllObservation().size() == 0) {
            DelMapPoint(mp);
            continue;
        } else {
            recent_created_mps.push_back(mp);
        }
    }

    Map::UpdateConnections(frame);
    for (auto kf : local_kfs) {
        Map::UpdateConnections(kf);
    }

    // local ba
    if (frame->id != 0) {
        // Perform local ba
        local_kfs = GrabLocalMap_Mappoint(frame, 3);
        std::unordered_set<KeyFrame> fix_local_kfs = GrabAnchorKeyFrames(local_kfs);
        local_kfs.insert(frame);
        fmt::print("local_kfs {}, fix_kfs {}\n", local_kfs.size(), fix_local_kfs.size());
        uint op_cnt = PoseEstimation::BoundleAdjustment(local_kfs, fix_local_kfs, 10, &PoseEstimation::FilterCallBack_Chi2);
        for (const KeyFrame& kf : local_kfs) {
            Map::UpdateConnections(kf);
        }
    }

    // update

    // UpdataConnections(frame);
}

void Map::DelKeyFrame(FrameRef frame) {
    assert(all_keyframes.count(frame) != 0);
    all_keyframes.erase(frame);
    // deleta all relative mappoints;
    for (auto obj : {frame->LEFT, frame->WIDE, frame->RIGHT})
        for (MapPointRef mpr : obj->GetMapPoints()) {
            mpr->UnBindKeyFrame(frame, obj);
        }

    {
        WRITELOCK _lock(mtx_connection);
        // clear all edge in connection graph
        if (essential_gragh.count(frame)) {
            for (const KeyFrame& _out_kf : essential_gragh[frame]) {
                essential_gragh[_out_kf].erase(frame);
            }
            essential_gragh.erase(frame);
        }
        // break spaning tree connection
        if (parent_kf.count(frame)) parent_kf.erase(frame);
    }
}

void Map::AddMapPoint(MapPointRef mp) { all_mappoints.insert(mp); }

void Map::DelMapPoint(MapPointRef mp) {
    assert(all_mappoints.count(mp) != 0);
    all_mappoints.erase(mp);
    for (auto ob : mp->GetAllObservation()) {
        KeyFrame kf = ob.first;
        for (ObjectRef obj : ob.second) {
            obj->DelMapPoint(mp);
        }
    }
}

void Map::Clear() {
    // 1. Must first free frames due to that The frame will automatically free relative mappoints
    for (KeyFrame kf : all_keyframes) {
        delete kf;
    }
    all_keyframes.clear();
    WRITELOCK _lock(mtx_connection);
    this->essential_gragh.clear();
    this->parent_kf.clear();
    this->loop_kfs.clear();
    // 2. Then , the mappoints can be free without mem leaky!
    all_mappoints.clear();
}

int Map::TrangularizationTwoObject(ObjectRef obj1, ObjectRef obj2, KeyFrame kf1, KeyFrame kf2, float min_baseline,
                                   const std::vector<float>& uright_obj1, const std::vector<float>& uright_obj2) {
    // bow match two frame

    float th = 0.5f;

    cv::Mat Rcw1 = obj1->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = obj1->GetTranslation();
    cv::Mat Tcw1(3, 4, CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0, 3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = obj1->GetCameraCenter();
    BaseCamera *pCamera1 = obj1->mpCam, *pCamera2 = obj2->mpCam;

    float fx1 = pCamera1->getParameter(CAM_PARA::FX);
    float fy1 = pCamera1->getParameter(CAM_PARA::FY);
    float cx1 = pCamera1->getParameter(CAM_PARA::CX);
    float cy1 = pCamera1->getParameter(CAM_PARA::CY);
    float invfx1 = pCamera1->getParameter(CAM_PARA::INVFX);
    float invfy1 = pCamera1->getParameter(CAM_PARA::INVFY);
    // Check first that baseline is not too short
    cv::Mat Ow2 = obj2->GetCameraCenter();

    cv::Mat vBaseline = Ow2 - Ow1;

    const float baseline = cv::norm(vBaseline);

    // median depth in obj2, why use obj2 not obj1 ?
    const float median_depth = obj2->ComputeSceneMedianDepth();
    if (baseline / median_depth < 0.01 || baseline / median_depth > 10) {
        fmt::print("Mapping::Trianglize:: traslation too less or large than baseline [{} < {}]\n", baseline, min_baseline);
        return 0;
    }

    // Compute Fundamental Matrix
    cv::Mat F12 = ComputeF12(obj1, obj2);

    auto match_res = Matcher::DBowMatch(obj1->desps, obj1->bow_feature, obj2->desps, obj2->bow_feature)
                         .FilterRatio(0.6)
                         .FilterThreshold(64)
                         .FilterOrientation(obj1->kps, obj2->kps)
                         .FilterFMatrix(obj1->kps, obj2->kps, F12, obj2->extractor->mvLevelSigma2);

    if (match_res.size() < 10) {
        return 0;
    }
    // Search matches that fullfil epipolar constraint
    vector<pair<size_t, size_t>> vMatchedIndices;

    cv::Mat Rcw2 = obj2->GetRotation();
    cv::Mat Rwc2 = Rcw2.t();
    cv::Mat tcw2 = obj2->GetTranslation();
    cv::Mat Tcw2(3, 4, CV_32F);
    Rcw2.copyTo(Tcw2.colRange(0, 3));
    tcw2.copyTo(Tcw2.col(3));

    const float fx2 = pCamera2->getParameter(CAM_PARA::FX);
    const float fy2 = pCamera2->getParameter(CAM_PARA::FY);
    const float cx2 = pCamera2->getParameter(CAM_PARA::CX);
    const float cy2 = pCamera2->getParameter(CAM_PARA::CY);
    const float invfx2 = pCamera2->getParameter(CAM_PARA::INVFX);
    const float invfy2 = pCamera2->getParameter(CAM_PARA::INVFY);

    int cnt = 0;
    // Triangulate each match
    for (const cv::DMatch& m : match_res) {
        const cv::KeyPoint& kp1 = obj1->kps[m.queryIdx];
        const cv::KeyPoint& kp2 = obj2->kps[m.trainIdx];

        float kp1_ur = -1, kp2_ur = -1;

        // Dawson Wen:
        // subpixel Match
        // kp2.pt += sub_pt2;

        // Check parallax between rays
        cv::Mat xn1 = pCamera1->unprojectMat(kp1.pt);
        cv::Mat xn2 = pCamera2->unprojectMat(kp2.pt);

        cv::Mat ray1 = Rwc1 * xn1;
        cv::Mat ray2 = Rwc2 * xn2;
        const float cosParallaxRays = ray1.dot(ray2) / (cv::norm(ray1) * cv::norm(ray2));

        float cosParallaxStereo = cosParallaxRays + 1;
        float cosParallaxStereo1 = cosParallaxStereo;
        float cosParallaxStereo2 = cosParallaxStereo;

        cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

        cv::Mat x3D;
        if (cosParallaxRays > 0 && ((cosParallaxRays < 0.9998))) {
            // Linear Triangulation Method
            cv::Mat A(4, 4, CV_32F);
            A.row(0) = xn1.at<float>(0) * Tcw1.row(2) - Tcw1.row(0);
            A.row(1) = xn1.at<float>(1) * Tcw1.row(2) - Tcw1.row(1);
            A.row(2) = xn2.at<float>(0) * Tcw2.row(2) - Tcw2.row(0);
            A.row(3) = xn2.at<float>(1) * Tcw2.row(2) - Tcw2.row(1);

            cv::Mat w, u, vt;
            cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

            x3D = vt.row(3).t();

            if (x3D.at<float>(3) == 0) continue;

            // Euclidean coordinates
            x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
        }

        //  else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2) {
        //     x3D = obj1->UnprojectStereo(idx1);
        // } else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1) {
        //     x3D = obj2->UnprojectStereo(idx2);
        // } else {
        //     continue;  // wide match and low parallax
        // }

        if (x3D.empty()) continue;
        cv::Mat x3Dt = x3D.t();

        // Check triangulation in front of cameras
        float z1 = Rcw1.row(2).dot(x3Dt) + tcw1.at<float>(2);
        if (z1 <= 0) continue;

        float z2 = Rcw2.row(2).dot(x3Dt) + tcw2.at<float>(2);
        if (z2 <= 0) continue;

        // Check reprojection error in first keyframe

        const float& sigmaSquare1 = obj1->extractor->mvLevelSigma2[kp1.octave];
        const float x1 = Rcw1.row(0).dot(x3Dt) + tcw1.at<float>(0);
        const float y1 = Rcw1.row(1).dot(x3Dt) + tcw1.at<float>(1);
        const float invz1 = 1.0 / z1;

        cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1, y1, z1));
        float errX1 = uv1.x - kp1.pt.x;
        float errY1 = uv1.y - kp1.pt.y;

        if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1) continue;

        // Check reprojection error in second keyframe
        const float sigmaSquare2 = obj2->extractor->mvLevelSigma2[kp2.octave];
        const float x2 = Rcw2.row(0).dot(x3Dt) + tcw2.at<float>(0);
        const float y2 = Rcw2.row(1).dot(x3Dt) + tcw2.at<float>(1);
        const float invz2 = 1.0 / z2;

        cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2, y2, z2));
        float errX2 = uv2.x - kp2.pt.x;
        float errY2 = uv2.y - kp2.pt.y;
        if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2) continue;

        // Check scale consistency
        float dist1 = cv::norm(x3D - Ow1);
        float dist2 = cv::norm(x3D - Ow2);

        if (dist1 == 0 || dist2 == 0) continue;

        // if (mbFarPoints && (dist1 >= 2 * mThFarPoints || dist2 >= 2 *
        // mThFarPoints)) // MODIFICATION 	continue;

        const float ratioDist = dist2 / dist1;
        const float ratioOctave = obj1->extractor->mvScaleFactor[kp1.octave] / obj2->extractor->mvScaleFactor[kp2.octave];

        if (ratioDist < 0.6 || ratioDist > 1.4) continue;

        // Triangulation is succesfull

        MapPointRef new_mp = CreateMappoint(x3D, obj2->desps.row(m.trainIdx), kf2->id);
        cnt++;
        obj1->AddMapPoint(new_mp, m.queryIdx);
        obj2->AddMapPoint(new_mp, m.trainIdx);

        new_mp->BindKeyFrame(kf1, obj1);
        new_mp->BindKeyFrame(kf2, obj2);

        new_mp->ComputeDistinctiveDescriptors();
        new_mp->UpdateNormalVector();

        cnt++;
    }

    return 0;
}

cv::Mat Map::ComputeF12(ObjectRef& rig1, ObjectRef& rig2) {
    cv::Mat R1w = rig1->GetRotation();
    cv::Mat t1w = rig1->GetTranslation();
    cv::Mat R2w = rig2->GetRotation();
    cv::Mat t2w = rig2->GetTranslation();

    cv::Mat R12 = R1w * R2w.t();
    cv::Mat t12 = -R1w * R2w.t() * t2w + t1w;

    cv::Mat t12x = Converter::toSkewSymmetricMatrix(t12);

    const cv::Mat& K1 = rig1->mpCam->toK();
    const cv::Mat& K2 = rig2->mpCam->toK();

    return K1.t().inv() * t12x * R12 * K2.inv();
}

std::vector<cv::Mat> Map::GetAllMappointsForShow() {
    std::vector<cv::Mat> ret;
    for (const MapPointRef& mpr : all_mappoints) {
        ret.push_back(mpr->position_w);
    }
    return ret;
}

int Map::MapPointSize() { return all_mappoints.size(); }

int Map::KeyFrameSize() { return all_keyframes.size(); }

void Map::UpdateConnections(KeyFrame kf) {
    WRITELOCK _lock(mtx_connection);
    std::unordered_map<KeyFrame, uint> kf_counter;
    std::unordered_set<MapPointRef> mps = kf->LEFT->GetMapPoints();

    for (const MapPointRef& mp : mps) {
        for (const auto& ob : mp->GetAllObservation()) {
            KeyFrame relative_kf = ob.first;
            if (relative_kf->id != kf->id) kf_counter[relative_kf] += 1;
        }
    }

    // This should not happen
    if (kf_counter.empty()) return;

    // clear all edge in connection graph
    if (essential_gragh.count(kf)) {
        for (const KeyFrame& _out_kf : essential_gragh[kf]) {
            essential_gragh[_out_kf].erase(kf);
        }
        essential_gragh.erase(kf);
    }
    // break spaning tree connection
    if (parent_kf.count(kf)) parent_kf.erase(kf);

    // If the counter is greater than threshold add connection
    // In case no keyframe counter is over threshold add the one with maximum
    // counter
    uint nmax = 0;
    KeyFrame max_conc_kf;
    bool added_conc = 0;
    for (const std::pair<const KeyFrame, uint>& p : kf_counter) {
        const KeyFrame _kf = p.first;
        uint vote_cnt = p.second;
        if (vote_cnt > nmax) {
            nmax = vote_cnt;
            max_conc_kf = _kf;
        }
        if (vote_cnt >= connection_threshold) {
            // add Connection
            essential_gragh[_kf].insert(kf);
            essential_gragh[kf].insert(_kf);
            added_conc += 1;
        }
    }

    if (added_conc == 0) {
        // if all connection is under threshold,
        // forcely create a connection between max_conc and this keyframe
        essential_gragh[max_conc_kf].insert(kf);
        essential_gragh[kf].insert(max_conc_kf);
    }

    if (kf->id == 0) {
        // No need to create ad parenet keyframe
    } else {
        parent_kf[kf] = max_conc_kf;
    }
}

void Map::AddLoopConnection(KeyFrame kf0, KeyFrame kf1) {
    WRITELOCK _lock(mtx_connection);
    loop_kfs[kf0].insert(kf1);
    loop_kfs[kf1].insert(kf0);
}

void Map::DelLoopConnection(KeyFrame kf0, KeyFrame kf1) {
    WRITELOCK _lock(mtx_connection);
    loop_kfs[kf0].erase(kf1);
    if (loop_kfs[kf0].size() == 0) {
        loop_kfs.erase(kf0);
    }
    loop_kfs[kf1].erase(kf0);
    if (loop_kfs[kf1].size() == 0) {
        loop_kfs.erase(kf1);
    }
}

std::unordered_set<KeyFrame> Map::GrabLocalMap_EssGraph(KeyFrame kf, uint hop) {
    assert(hop >= 1);

    READLOCK _lock(mtx_connection);
    std::unordered_set<KeyFrame> ret;
    ret.insert(kf);

    std::unordered_set<KeyFrame> cur_hop = ret;
    while (hop-- && cur_hop.size()) {
        std::unordered_set<KeyFrame> next_hop;
        for (KeyFrame _kf : cur_hop) {
            for (KeyFrame _to_kf : essential_gragh[_kf]) {
                if (ret.count(_to_kf) == 0) {
                    ret.insert(_to_kf);
                    next_hop.insert(_to_kf);
                }
            }
        }
        cur_hop = next_hop;
    }
    if (ret.count(kf) != 0) ret.erase(kf);
    return ret;
}

std::unordered_set<KeyFrame> Map::GrabLocalMap_Mappoint(FrameRef frame, uint hop) {
    assert(hop >= 1);
    hop--;
    std::unordered_set<KeyFrame> ret;
    for (MapPointRef mp : frame->LEFT->GetMapPoints()) {
        for (const pair<const KeyFrame, unordered_set<ObjectRef>>& p : mp->GetAllObservation()) {
            ret.insert(p.first);
        }
    }

    READLOCK _lock(mtx_connection);
    std::unordered_set<KeyFrame> cur_hop = ret;
    while (hop-- && cur_hop.size()) {
        std::unordered_set<KeyFrame> next_hop;
        for (KeyFrame _kf : cur_hop) {
            for (KeyFrame _to_kf : essential_gragh[_kf]) {
                if (ret.count(_to_kf) == 0) {
                    ret.insert(_to_kf);
                    next_hop.insert(_to_kf);
                }
            }
        }
        cur_hop = next_hop;
    }
    if (ret.count(frame) != 0) ret.erase(frame);
    return ret;
}

std::unordered_set<MapPointRef> Map::GrabLocalMappoint(const std::unordered_set<KeyFrame>& local_kfs, CAM_NAME name) {
    std::unordered_set<MapPointRef> ret;
    for (const KeyFrame& kf : local_kfs) {
        if (name & CAM_NAME::L) {
            for (const auto& mp : kf->LEFT->GetMapPoints()) {
                ret.insert(mp);
            }
        }
        if (name & CAM_NAME::W) {
            for (const auto& mp : kf->WIDE->GetMapPoints()) {
                ret.insert(mp);
            }
        }
    }
    return ret;
}

std::unordered_set<KeyFrame> Map::GrabAnchorKeyFrames(std::unordered_set<KeyFrame>& kfs) {
    // find the keyframe with minmial id
    KeyFrame kf_min_id = nullptr;
    for (KeyFrame _kf : kfs) {
        if (kf_min_id == nullptr || kf_min_id->id > _kf->id) {
            kf_min_id = _kf;
        }
    }
    if (kf_min_id == nullptr) {
        return {};
    }

    std::unordered_set<KeyFrame> kfs_min_id_one_hop = GrabLocalMap_EssGraph(kf_min_id, 1);
    std::unordered_set<KeyFrame> kfs_anchor;
    // find the keyframes which id is small than kfs' minmal id;
    for (KeyFrame _kf : kfs_min_id_one_hop) {
        if (_kf->id < kf_min_id->id) {
            kfs_anchor.insert(_kf);
        }
    }
    return kfs_anchor;
}

}  // namespace MCVSLAM