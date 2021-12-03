/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CAMERAMODELS_Pinhole_H
#define CAMERAMODELS_Pinhole_H
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "BaseCamera.hpp"
namespace MCVSLAM {

enum CAM_NAME { L = 0x01, R = 0x02, W = 0x04 };

class Pinhole : public BaseCamera {
   public:
    Pinhole(const std::string &config_file);
    Pinhole(const std::vector<float> &_vParameters, cv::Mat _distCoef) : BaseCamera(_vParameters, _distCoef) {}
    ~Pinhole() {}

    cv::Point2f project(const cv::Point3f &p3D);
    cv::Point2f project(const cv::Mat &m3D);
    Eigen::Vector2d project(const Eigen::Vector3d &v3D);
    cv::Mat projectMat(const cv::Point3f &p3D);

    cv::Mat unproject_z(const cv::Point2f &p2D, float z);

    cv::Point3f unproject(const cv::Point2f &p2D);
    cv::Mat unprojectMat(const cv::Point2f &p2D);

    cv::Mat projectJac(const cv::Point3f &p3D);
    Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d &v3D);

    cv::Mat unprojectJac(const cv::Point2f &p2D);
};
}  // namespace MCVSLAM

#endif  // CAMERAMODELS_Pinhole_H
