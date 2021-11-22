/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Pinhole.hpp"

#include <opencv2/core/hal/interface.h>

#include "BaseCamera.hpp"
#include "pyp/yaml/yaml.hpp"
namespace MCVSLAM {

Pinhole::Pinhole(const std::string &config_file) : BaseCamera() {
    Yaml::Node root;
    Yaml::Parse(root, config_file);
    parameters = root["intrisic"].AsVector<float>();
    std::vector<float> vK = root["K"].AsVector<float>();
    assert(vK.size() == 4);
    K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = vK[0];
    K.at<float>(1, 1) = vK[1];
    K.at<float>(0, 2) = vK[2];
    K.at<float>(1, 2) = vK[3];
    std::cout << K << std::endl;
    std::vector<float> vDist = root["distort"].AsVector<float>();
    assert(vDist.size() == 5);
    dist = cv::Mat(vDist).reshape(0, 1).clone();
    std::cout << dist << std::endl;
}

cv::Point2f Pinhole::project(const cv::Point3f &p3D) {
    return cv::Point2f(parameters[0] * p3D.x / p3D.z + parameters[2], parameters[1] * p3D.y / p3D.z + parameters[3]);
}

cv::Point2f Pinhole::project(const cv::Mat &m3D) {
    const float *p3D = m3D.ptr<float>();

    return this->project(cv::Point3f(p3D[0], p3D[1], p3D[2]));
}

Eigen::Vector2d Pinhole::project(const Eigen::Vector3d &v3D) {
    Eigen::Vector2d res;
    res[0] = parameters[0] * v3D[0] / v3D[2] + parameters[2];
    res[1] = parameters[1] * v3D[1] / v3D[2] + parameters[3];

    return res;
}

cv::Mat Pinhole::projectMat(const cv::Point3f &p3D) {
    cv::Point2f point = this->project(p3D);
    return (cv::Mat_<float>(2, 1) << point.x, point.y);
}

cv::Point3f Pinhole::unproject(const cv::Point2f &p2D) {
    return cv::Point3f((p2D.x - parameters[2]) / parameters[0], (p2D.y - parameters[3]) / parameters[1], 1.f);
}

cv::Mat Pinhole::unproject_z(const cv::Point2f &p2D, float z) {
    float fx = getParameter(CAM_PARA::FX);
    float fy = getParameter(CAM_PARA::FY);
    float cx = getParameter(CAM_PARA::CX);
    float cy = getParameter(CAM_PARA::CY);
    float invfx = getParameter(CAM_PARA::INVFX);
    float invfy = getParameter(CAM_PARA::INVFY);

    const float u = p2D.x;
    const float v = p2D.y;
    const float x = (u - cx) * z * invfx;
    const float y = (v - cy) * z * invfy;
    cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
    return x3Dc;
}

cv::Mat Pinhole::unprojectMat(const cv::Point2f &p2D) {
    cv::Point3f ray = this->unproject(p2D);
    return (cv::Mat_<float>(3, 1) << ray.x, ray.y, ray.z);
}

cv::Mat Pinhole::projectJac(const cv::Point3f &p3D) {
    cv::Mat Jac(2, 3, CV_32F);
    Jac.at<float>(0, 0) = parameters[0] / p3D.z;
    Jac.at<float>(0, 1) = 0.f;
    Jac.at<float>(0, 2) = -parameters[0] * p3D.x / (p3D.z * p3D.z);
    Jac.at<float>(1, 0) = 0.f;
    Jac.at<float>(1, 1) = parameters[1] / p3D.z;
    Jac.at<float>(1, 2) = -parameters[1] * p3D.y / (p3D.z * p3D.z);

    return Jac;
}

Eigen::Matrix<double, 2, 3> Pinhole::projectJac(const Eigen::Vector3d &v3D) {
    Eigen::Matrix<double, 2, 3> Jac;
    Jac(0, 0) = parameters[0] / v3D[2];
    Jac(0, 1) = 0.f;
    Jac(0, 2) = -parameters[0] * v3D[0] / (v3D[2] * v3D[2]);
    Jac(1, 0) = 0.f;
    Jac(1, 1) = parameters[1] / v3D[2];
    Jac(1, 2) = -parameters[1] * v3D[1] / (v3D[2] * v3D[2]);

    return Jac;
}

cv::Mat Pinhole::unprojectJac(const cv::Point2f &p2D) {
    cv::Mat Jac(3, 2, CV_32F);
    Jac.at<float>(0, 0) = 1 / parameters[0];
    Jac.at<float>(0, 1) = 0.f;
    Jac.at<float>(1, 0) = 0.f;
    Jac.at<float>(1, 1) = 1 / parameters[1];
    Jac.at<float>(2, 0) = 0.f;
    Jac.at<float>(2, 1) = 0.f;

    return Jac;
}

}  // namespace MCVSLAM