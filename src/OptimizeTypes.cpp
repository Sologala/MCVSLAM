#include "OptimizeTypes.hpp"

namespace MCVSLAM {

EdgeStereoSE3ProjectXYZ::EdgeStereoSE3ProjectXYZ(BaseCamera *_cam, const double _bf) : bf(_bf) {
    fx = _cam->getParameter(CAM_PARA::FX);
    fy = _cam->getParameter(CAM_PARA::FY);
    cx = _cam->getParameter(CAM_PARA::CX);
    cy = _cam->getParameter(CAM_PARA::CY);
}

void EdgeStereoSE3ProjectXYZ::linearizeOplus() {
    g2o::VertexSE3Expmap *Tcw = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
    g2o::SE3Quat T(Tcw->estimate());
    g2o::VertexSBAPointXYZ *Pw = static_cast<g2o::VertexSBAPointXYZ *>(_vertices[0]);
    Eigen::Vector3d xyz = Pw->estimate();
    Eigen::Vector3d xyz_trans = T.map(xyz);

    const Eigen::Matrix3d R = T.rotation().toRotationMatrix();

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double z_2 = z * z;

    _jacobianOplusXi(0, 0) = -fx * R(0, 0) / z + fx * x * R(2, 0) / z_2;
    _jacobianOplusXi(0, 1) = -fx * R(0, 1) / z + fx * x * R(2, 1) / z_2;
    _jacobianOplusXi(0, 2) = -fx * R(0, 2) / z + fx * x * R(2, 2) / z_2;

    _jacobianOplusXi(1, 0) = -fy * R(1, 0) / z + fy * y * R(2, 0) / z_2;
    _jacobianOplusXi(1, 1) = -fy * R(1, 1) / z + fy * y * R(2, 1) / z_2;
    _jacobianOplusXi(1, 2) = -fy * R(1, 2) / z + fy * y * R(2, 2) / z_2;

    _jacobianOplusXi(2, 0) = _jacobianOplusXi(0, 0) - bf * R(2, 0) / z_2;
    _jacobianOplusXi(2, 1) = _jacobianOplusXi(0, 1) - bf * R(2, 1) / z_2;
    _jacobianOplusXi(2, 2) = _jacobianOplusXi(0, 2) - bf * R(2, 2) / z_2;

    _jacobianOplusXj(0, 0) = x * y / z_2 * fx;
    _jacobianOplusXj(0, 1) = -(1 + (x * x / z_2)) * fx;
    _jacobianOplusXj(0, 2) = y / z * fx;
    _jacobianOplusXj(0, 3) = -1. / z * fx;
    _jacobianOplusXj(0, 4) = 0;
    _jacobianOplusXj(0, 5) = x / z_2 * fx;

    _jacobianOplusXj(1, 0) = (1 + y * y / z_2) * fy;
    _jacobianOplusXj(1, 1) = -x * y / z_2 * fy;
    _jacobianOplusXj(1, 2) = -x / z * fy;
    _jacobianOplusXj(1, 3) = 0;
    _jacobianOplusXj(1, 4) = -1. / z * fy;
    _jacobianOplusXj(1, 5) = y / z_2 * fy;

    _jacobianOplusXj(2, 0) = _jacobianOplusXj(0, 0) - bf * y / z_2;
    _jacobianOplusXj(2, 1) = _jacobianOplusXj(0, 1) + bf * x / z_2;
    _jacobianOplusXj(2, 2) = _jacobianOplusXj(0, 2);
    _jacobianOplusXj(2, 3) = _jacobianOplusXj(0, 3);
    _jacobianOplusXj(2, 4) = 0;
    _jacobianOplusXj(2, 5) = _jacobianOplusXj(0, 5) - bf / z_2;
}
}  // namespace MCVSLAM
