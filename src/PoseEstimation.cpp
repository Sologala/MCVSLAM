
#include "PoseEstimation.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <opencv2/core/hal/interface.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>

#include "BAoptimizer.hpp"
#include "Converter.h"
#include "Map.hpp"
#include "MapPoint.hpp"
#include "Object.hpp"
#include "Pinhole.hpp"
#include "g2o/types/slam3d/edge_se3.h"

using namespace std;
using namespace MCVSLAM;
using namespace g2o;

cv::Mat MCVSLAM::PoseEstimation::_2d2d(const ObjectRef &obj1, const ObjectRef &obj2, const std::vector<cv::DMatch> &match_res, uint method) {
    std::vector<cv::Point2f> pts1, pts2;
    pts1.reserve(obj1->size());
    pts2.reserve(obj2->size());
    for (uint i = 0, sz = match_res.size(); i < sz; i++) {
        pts1.push_back(obj1->kps[match_res[i].queryIdx].pt);
        pts2.push_back(obj2->kps[match_res[i].trainIdx].pt);
    }

    cv::Point2d pp(obj1->mpCam->getParameter(CAM_PARA::CX), obj1->mpCam->getParameter(CAM_PARA::CY));
    float focal_length = obj1->mpCam->getParameter(CAM_PARA::FX);
    cv::Mat mask;
    cv::Mat E_mat = cv::findEssentialMat(pts1, pts2, focal_length, pp, cv::RANSAC, 0.999, 1.0, mask);
    cv::Mat T12 = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat R, t;
    cv::recoverPose(E_mat, pts1, pts2, R, t, focal_length, pp, mask);
    R.copyTo(T12.rowRange(0, 3).colRange(0, 3));
    t.copyTo(T12.rowRange(0, 3).col(3));
    return T12;
}

int MCVSLAM::PoseEstimation::PoseOptimization(const KeyFrame &frame) {
    BAoptimizer op;
    int nInitialCorrespondences = 0;
    // Set Frame vertex
    op.addKeyFrame(frame);
    {
        bool fixed = true;
        for (MapPointRef mp : frame->LEFT->GetMapPoints()) {
            op.addMapppint(mp, fixed);
            // Monocular observation
            uint idx = frame->LEFT->GetMapPointIdx(mp);
            cv::KeyPoint kp = frame->LEFT->kps[idx];
            if (frame->u_right[idx] != -1 && frame->u_right[idx] >= MIN_DISPARITY) {
                op.addEdgeStereo(frame, mp, kp.pt, frame->u_right[idx], kp.octave);
            } else  // Stereo observation
            {
                op.addEdge(frame, mp, kp.pt, kp.octave);
            }
        }

        for (MapPointRef mp : frame->WIDE->GetMapPoints()) {
            op.addMapppint(mp, fixed);
            uint idx = frame->WIDE->GetMapPointIdx(mp);
            cv::KeyPoint kp = frame->WIDE->kps[idx];
            op.addEdgeToBody(frame, mp, kp.pt, kp.octave);
        }
    }

    nInitialCorrespondences = op.getEdgeSize();

    if (nInitialCorrespondences < 3) return 0;

    // We perform 4 optimizations, after each optimization we classify observation
    // as inlier/outlier At the next optimization, outliers are not included, but
    // at the end they can be classified as inliers again.
    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
    const int its[4] = {15, 15, 15, 15};

    int nBad = 0;
    int nleft, nwide = 0;
    srand(time(0));

    for (size_t it = 0; it < 4; it++) {
        // if (rand() % 2 == 0)
        // 	op.vertex(0)->setEstimate(Converter::toSE3Quat(pFrame->GetPose(CAMNAME::L)));
        op.optimize(its[it]);
        nBad = 0;

        // filter call back
        auto filter_callback = [&it, &chi2Stereo, &chi2Mono, &nBad](BAoptimizer::EdgeInfoMation info) {
            auto e = info.first;
            MapPointRef mp = info.second.first;
            ObjectRef obj = info.second.second;
            if (e->level() == 1) {
                // level == 1 means this edge has been abort
                return;
            }
            e->computeError();
            float chi2 = e->chi2();
            const float thres_hold = (obj->name == CAM_NAME::R ? chi2Stereo[it] : chi2Mono[it]);
            if (chi2 >= thres_hold) {
                e->setLevel(1);
                mp->ProjectResRecord(false);
                obj->DelMapPoint(mp);
                nBad++;
            }
            if (it == 2) e->setRobustKernel(0);
        };
        op.filter(filter_callback);

        if (nInitialCorrespondences - nBad < 10) break;
    }
    if (nInitialCorrespondences - nBad < 10) {
        fmt::print("Too few edge to optimize current pose , Discarded !\n ");
    } else {
        frame->SetPose(op.eval(frame));
    }

    op.DumpHessian("asad");

    return nInitialCorrespondences - nBad;
}

bool MCVSLAM::PoseEstimation::FilterCallBack_Chi2(const BAoptimizer::EdgeInfoMation &info) {
    // std::unordered_set<MapPointRef> bad_mps;
    auto e = info.first;
    MapPointRef mp = info.second.first;
    ObjectRef obj = info.second.second;
    if (e->level() == 1) {
        // level == 1 means this edge has been abort
        return false;
    }
    e->computeError();
    float chi2 = e->chi2();
    const float thres_hold = (obj->name == CAM_NAME::R ? CHI2_STEREO_THRESHOLD : CHI2_MONO_THRESHOLD);
    if (chi2 >= thres_hold) {
        obj->DelMapPoint(mp);
        mp->ProjectResRecord(false);
        e->setLevel(1);
        return true;
    }
    return false;
}

bool MCVSLAM::PoseEstimation::ThisMapPointBeFix(MapPointRef mp, const std::unordered_set<KeyFrame> &kfs,
                                                const std::unordered_set<KeyFrame> &fix_kfs) {
    uint cnt_fix = 0, cnt_no_fix = 0;
    for (auto _relative_kf : mp->GetAllKeyFrame()) {
        if (kfs.count(_relative_kf))
            cnt_no_fix += 1;
        else if (fix_kfs.count(_relative_kf))
            cnt_fix += 1;
    }
    return (cnt_no_fix + 1) * 1. / (cnt_fix + 1) > 1.5 ? false : true;
}
