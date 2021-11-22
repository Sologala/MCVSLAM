#ifndef BASECAMERA_H
#define BASECAMERA_H
#pragma once
#include <Eigen/Geometry>
#include <eigen3/Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace MCVSLAM {
enum CAM_PARA { FX = 0, FY = 1, CX = 2, CY = 3, INVFX = 4, INVFY = 5 };
class BaseCamera {
   public:
    BaseCamera() : parameters(6, 0){};
    BaseCamera(const std::vector<float> &_parameters, cv::Mat _dist) : parameters(_parameters), dist(_dist) {
        parameters.push_back(1.0 / parameters[FX]);
        parameters.push_back(1.0 / parameters[FY]);
        K = (cv::Mat_<float>(3, 3) << parameters[0], 0.f, parameters[2], 0.f, parameters[1], parameters[3], 0.f, 0.f, 1.f);
    };
    virtual ~BaseCamera(){};

    virtual cv::Point2f project(const cv::Point3f &p3D) = 0;
    virtual cv::Point2f project(const cv::Mat &m3D) = 0;
    virtual Eigen::Vector2d project(const Eigen::Vector3d &v3D) = 0;
    virtual cv::Mat projectMat(const cv::Point3f &p3D) = 0;

    virtual cv::Mat unproject_z(const cv::Point2f &p2D, float z) = 0;

    virtual cv::Point3f unproject(const cv::Point2f &p2D) = 0;
    virtual cv::Mat unprojectMat(const cv::Point2f &p2D) = 0;

    virtual cv::Mat projectJac(const cv::Point3f &p3D) = 0;
    virtual Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d &v3D) = 0;

    virtual cv::Mat unprojectJac(const cv::Point2f &p2D) = 0;

    float getParameter(const CAM_PARA i) { return parameters[i]; }
    float getParameter(const int i) { return parameters[i]; }
    cv::Mat getDistCoef() { return dist; }
    cv::Mat toK() { return K; }

   public:
    std::vector<float> parameters;
    cv::Mat dist;
    cv::Mat K;
};

}  // namespace MCVSLAM

#endif