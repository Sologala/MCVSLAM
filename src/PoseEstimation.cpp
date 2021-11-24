#include "PoseEstimation.hpp"

#include <opencv2/core/hal/interface.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
using namespace std;
using namespace MCVSLAM;

cv::Mat MCVSLAM::PoseEstimation::_2d2d(ObjectRef obj1, ObjectRef obj2, const std::vector<cv::DMatch>& match_res, uint method) {
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
