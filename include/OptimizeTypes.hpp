
// #include
#ifndef OPTIMIZETYPES_HPP
#define OPTIMIZETYPES_HPP

#include <memory>
#pragma once
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/sim3.h>

#include <Eigen/Geometry>
#include <Pinhole.hpp>

#include "Converter.h"
namespace MCVSLAM {
class EdgeSE3XYZ2CameraOnlyPose : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3XYZ2CameraOnlyPose(const cv::Mat Tcb, const cv::Mat XYZ, BaseCamera *_pCamera)
        : Pw(Converter::toVector3d(XYZ)), Tcb(Converter::toSE3Quat(Tcb)), cam(_pCamera) {}
    virtual ~EdgeSE3XYZ2CameraOnlyPose(){};
    bool read(std::istream &is) { return true; }

    bool write(std::ostream &os) const { return true; }

    void computeError() {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs - cam->project((Tcb * v1->estimate()).map(Pw));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        return ((Tcb * v1->estimate()).map(Pw))(2) > 0.0;
    }

    void linearizeOplus() {
        g2o::VertexSE3Expmap *v = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat Tbody_world(v->estimate());
        // P' = T x
        // world to body
        Eigen::Vector3d X_body = Tbody_world.map(Pw);
        // body to camera
        Eigen::Vector3d X_c = Tcb.map(X_body);

        double x_w = X_body[0];
        double y_w = X_body[1];
        double z_w = X_body[2];

        Eigen::Matrix<double, 3, 6> SE3deriv;
        // se3 定义是旋转在前 平移在后面。
        // 这里的导数是对 body 的 SE3 pose 进行求导。

        SE3deriv << 0.f, z_w, -y_w, 1.f, 0.f, 0.f, -z_w, 0.f, x_w, 0.f, 1.f, 0.f, y_w, -x_w, 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXi = -cam->projectJac(X_c) * Tcb.rotation().toRotationMatrix() * SE3deriv;
    }

   public:
    Eigen::Vector3d Pw;
    BaseCamera *cam;
    g2o::SE3Quat Tcb;
};

class EdgeSE3XYZ2Camera : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3XYZ2Camera(const cv::Mat &_Tcb, BaseCamera *_camera) : Tcb(Converter::toSE3Quat(_Tcb)), cam(_camera) {}
    virtual ~EdgeSE3XYZ2Camera(){};
    bool read(std::istream &is) { return true; }

    bool write(std::ostream &os) const { return true; }

    void computeError() {
        const g2o::VertexSE3Expmap *Tbw = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *Pw = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        // camera <- body <- world
        _error = obs - cam->project((Tcb * Tbw->estimate()).map(Pw->estimate()));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap *Tbw = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *Pw = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        return ((Tcb * Tbw->estimate()).map(Pw->estimate()))(2) > 0.0;
    }

    void linearizeOplus() {
        g2o::VertexSE3Expmap *Tbw = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T_body_world(Tbw->estimate());
        g2o::SE3Quat T_cam_world = Tcb * T_body_world;
        g2o::VertexSBAPointXYZ *Pw = static_cast<g2o::VertexSBAPointXYZ *>(_vertices[0]);
        Eigen::Vector3d P_world = Pw->estimate();
        Eigen::Vector3d P_body = T_body_world.map(P_world);
        Eigen::Vector3d P_cam = Tcb.map(T_body_world.map(P_world));

        _jacobianOplusXi = -cam->projectJac(P_cam) * T_cam_world.rotation().toRotationMatrix();

        double x = P_body[0];
        double y = P_body[1];
        double z = P_body[2];

        Eigen::Matrix<double, 3, 6> SE3deriv;
        SE3deriv << 0.f, z, -y, 1.f, 0.f, 0.f, -z, 0.f, x, 0.f, 1.f, 0.f, y, -x, 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXj = -cam->projectJac(P_cam) * Tcb.rotation().toRotationMatrix() * SE3deriv;
    }

    BaseCamera *cam;
    g2o::SE3Quat Tcb;
};

class EdgeStereoSE3ProjectXYZ : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeStereoSE3ProjectXYZ(BaseCamera *_cam, const double _bf);
    virtual ~EdgeStereoSE3ProjectXYZ() {}

    bool read(std::istream &is) { return true; };

    bool write(std::ostream &os) const { return true; };

    void computeError() {
        const g2o::VertexSE3Expmap *Tcw = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *Pw = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        Eigen::Vector3d obs(_measurement);
        Eigen::Vector3d P_camera = Tcw->estimate().map(Pw->estimate());
        _error = obs - cam_project_uv_u(P_camera, bf);
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap *Tcw = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *Pw = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        return (Tcw->estimate().map(Pw->estimate()))(2) > 0.0;
    }

    virtual void linearizeOplus();

    Eigen::Vector3d cam_project_uv_u(const Eigen::Vector3d &trans_xyz, const float &bf) const {
        const float invz = 1.0f / trans_xyz[2];
        Eigen::Vector3d res;
        res[0] = trans_xyz[0] * invz * fx + cx;
        res[1] = trans_xyz[1] * invz * fy + cy;
        // this dimention is the u in the right camera
        // bf * invz means the parallax
        res[2] = res[0] - bf * invz;
        return res;
    };
    double fx, fy, cx, cy, bf;
};
}  // namespace MCVSLAM
#endif
