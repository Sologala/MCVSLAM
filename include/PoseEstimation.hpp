#ifndef POSEESTIMATION_H
#define POSEESTIMATION_H
#include <g2o/core/base_edge.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>

#include "BAoptimizer.hpp"
#include "Frame.hpp"
#include "MapPoint.hpp"
#include "Object.hpp"
#pragma once
namespace MCVSLAM {
#define MIN_DISPARITY 1
#define CHI2_STEREO_THRESHOLD 7.815
#define CHI2_MONO_THRESHOLD 5.991
class PoseEstimation {
   public:
    PoseEstimation(){};
    ~PoseEstimation(){};

    // estimate pose with respect to obj1 pose
    static cv::Mat _2d2d(const ObjectRef& obj1, const ObjectRef& obj2, const std::vector<cv::DMatch>& match_res, uint method = cv::FM_8POINT);
    static int PoseOptimization(const KeyFrame& frame);

    template <typename Filter_CallBack>
    static int BoundleAdjustment(const std::unordered_set<KeyFrame>& kfs, const std::unordered_set<KeyFrame>& fix_kfs, uint n_iter,
                                 Filter_CallBack&& Filter_CALLBACK);
    // static cv::Mat _2d2d(ObjectRef obj1, ObjectRef obj2, const std::vector<cv::DMatch>& match_res);
    static bool FilterCallBack_Chi2(const BAoptimizer::EdgeInfoMation& info);

    static bool ThisMapPointBeFix(MapPointRef mp, const std::unordered_set<KeyFrame>& kfs, const std::unordered_set<KeyFrame>& fix_kfs);
};

template <typename Filter_CallBack>
inline int PoseEstimation::BoundleAdjustment(const std::unordered_set<KeyFrame>& kfs, const std::unordered_set<KeyFrame>& fix_kfs, uint n_iter,
                                             Filter_CallBack&& Filter_CALLBACK) {
    BAoptimizer op;
    uint cnt_mp_fix = 0, cnt_mp_nfix = 0;
    for (const KeyFrame& kf : fix_kfs) {
        op.addKeyFrame(kf, true);
        for (MapPointRef mp : kf->LEFT->GetMapPoints()) {
            if (ThisMapPointBeFix(mp, kfs, fix_kfs))
                cnt_mp_fix += 1;
            else
                cnt_mp_nfix += 1;

            op.addMapppint(mp, ThisMapPointBeFix(mp, kfs, fix_kfs));
            // Monocular observation
            uint idx = kf->LEFT->GetMapPointIdx(mp);
            if (idx == -1) continue;

            cv::KeyPoint kp = kf->LEFT->kps[idx];
            if (kf->u_right[idx] != -1 && kf->u_right[idx] >= MIN_DISPARITY) {
                op.addEdgeStereo(kf, mp, kp.pt, kf->u_right[idx], kp.octave);
            } else  // Stereo observation
            {
                op.addEdge(kf, mp, kp.pt, kp.octave);
            }
        }

        for (MapPointRef mp : kf->WIDE->GetMapPoints()) {
            op.addMapppint(mp, ThisMapPointBeFix(mp, kfs, fix_kfs));
            uint idx = kf->WIDE->GetMapPointIdx(mp);
            if (idx == -1) continue;

            if (ThisMapPointBeFix(mp, kfs, fix_kfs))
                cnt_mp_fix += 1;
            else
                cnt_mp_nfix += 1;
            cv::KeyPoint kp = kf->WIDE->kps[idx];
            op.addEdgeToBody(kf, mp, kp.pt, kp.octave);
        }
    }
    for (const KeyFrame& kf : kfs) {
        op.addKeyFrame(kf, false);
        for (MapPointRef mp : kf->LEFT->GetMapPoints()) {
            if (ThisMapPointBeFix(mp, kfs, fix_kfs))
                cnt_mp_fix += 1;
            else
                cnt_mp_nfix += 1;
            op.addMapppint(mp, ThisMapPointBeFix(mp, kfs, fix_kfs));
            // Monocular observation
            uint idx = kf->LEFT->GetMapPointIdx(mp);
            if (idx == -1) continue;

            cv::KeyPoint kp = kf->LEFT->kps[idx];
            if (kf->u_right[idx] != -1 && kf->u_right[idx] >= MIN_DISPARITY) {
                op.addEdgeStereo(kf, mp, kp.pt, kf->u_right[idx], kp.octave);
            } else  // Stereo observation
            {
                op.addEdge(kf, mp, kp.pt, kp.octave);
            }
        }

        for (MapPointRef mp : kf->WIDE->GetMapPoints()) {
            if (ThisMapPointBeFix(mp, kfs, fix_kfs))
                cnt_mp_fix += 1;
            else
                cnt_mp_nfix += 1;
            op.addMapppint(mp, ThisMapPointBeFix(mp, kfs, fix_kfs));
            uint idx = kf->WIDE->GetMapPointIdx(mp);
            if (idx == -1) continue;

            cv::KeyPoint kp = kf->WIDE->kps[idx];
            op.addEdgeToBody(kf, mp, kp.pt, kp.octave);
        }
    }
    fmt::print("fix mps {} no fix mp {}  \n", cnt_mp_fix, cnt_mp_nfix);

    op.optimize(n_iter);

    // filter call back
    uint init_cnt_edge = op.getEdgeSize();
    uint nBad = 0;

    auto filter_back_wraper = [&nBad, &Filter_CALLBACK](BAoptimizer::EdgeInfoMation info) {
        if (Filter_CALLBACK(info)) nBad += 1;
    };
    op.filter(Filter_CALLBACK);
    uint success_edge = init_cnt_edge - nBad;
    if (success_edge > 10) {
        op.recovery_all();
    }
    return success_edge;
}
}  // namespace MCVSLAM
#endif