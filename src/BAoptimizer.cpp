#include "BAoptimizer.hpp"

#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/stuff/misc.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <opencv2/core/hal/interface.h>

#include <memory>

#include "Map.hpp"
namespace MCVSLAM {
BAoptimizer::BAoptimizer() {
    linearSolver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
    _vettex_id = 0;
    this->setAlgorithm(solver);
}

BAoptimizer::~BAoptimizer() {}

void BAoptimizer::addKeyFrame(const KeyFrame &kf, bool fixed) {
    if (KF2ID.count(kf) != 0) return;
    KF2ID[kf] = _vettex_id++;
    ALLKFS.insert(kf);
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(kf->GetPose()));
    vSE3->setId(KF2ID[kf]);
    // Set fixed if kf is the first frame.
    vSE3->setFixed(fixed || kf->id == 0);
    g2o::SparseOptimizer::addVertex(vSE3);
}
void BAoptimizer::addMapppint(const MapPointRef &mp, bool fixed) {
    if (MP2ID.count(mp) != 0) return;
    MP2ID[mp] = _vettex_id++;
    ALLMPS.insert(mp);
    g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(Converter::toVector3d(mp->GetWorldPos()));
    vPoint->setId(MP2ID[mp]);
    vPoint->setFixed(fixed);
    vPoint->setMarginalized(true);
    g2o::SparseOptimizer::addVertex(vPoint);
}

void BAoptimizer::addEdge(const KeyFrame &kf, const MapPointRef &mp, const cv::Point2f &uv, const float &level) {
    assert(KF2ID.count(kf) != 0);

    assert(MP2ID.count(mp) != 0);

    Eigen::Matrix<double, 2, 1> obs;
    obs << uv.x, uv.y;
    EdgeSE3XYZ2Camera *e = new EdgeSE3XYZ2Camera(cv::Mat::eye(4, 4, CV_32F), kf->LEFT->mpCam);

    e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex *>(vertex(MP2ID[mp])));
    e->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex *>(vertex(KF2ID[kf])));
    e->setMeasurement(obs);
    const float &invSigma2 = kf->extractor_left.mvInvLevelSigma2[level];
    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);
    rk->setDelta(thHuber2D);
    g2o::SparseOptimizer::addEdge(e);
    EdgeInfos[e] = {mp, kf->LEFT};
}

void BAoptimizer::addEdgeStereo(const KeyFrame &kf, const MapPointRef &mp, const cv::Point2f &uv, float ru, const float &level) {
    assert(KF2ID.count(kf) != 0);
    assert(MP2ID.count(mp) != 0);
    Eigen::Matrix<double, 3, 1> obs;

    obs << uv.x, uv.y, ru;

    EdgeStereoSE3ProjectXYZ *e = new EdgeStereoSE3ProjectXYZ(kf->LEFT->mpCam, kf->bf);

    e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex *>(vertex(MP2ID[mp])));
    e->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex *>(vertex(KF2ID[kf])));
    e->setMeasurement(obs);
    const float &invSigma2 = kf->extractor_left.mvInvLevelSigma2[level];
    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
    e->setInformation(Info);

    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);
    rk->setDelta(thHuber3D);
    g2o::SparseOptimizer::addEdge(e);
    EdgeInfos[e] = {mp, kf->LEFT};
}

void BAoptimizer::addEdgeToBody(const KeyFrame &kf, const MapPointRef &mp, const cv::Point2f &uv, const float &level) {
    assert(KF2ID.count(kf) != 0);
    assert(MP2ID.count(mp) != 0);

    Eigen::Matrix<double, 2, 1> obs;
    obs << uv.x, uv.y;

    EdgeSE3XYZ2Camera *e = new EdgeSE3XYZ2Camera(kf->Twl, kf->WIDE->mpCam);
    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vertex(MP2ID[mp])));
    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vertex(KF2ID[kf])));
    e->setMeasurement(obs);
    const float &invSigma2 = kf->extractor_wide.mvInvLevelSigma2[level];
    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);
    rk->setDelta(thHuber3D);
    g2o::SparseOptimizer::addEdge(e);
    EdgeInfos[e] = {mp, kf->LEFT};
}

void BAoptimizer::optimize(const int n) {
    g2o::SparseOptimizer::initializeOptimization();
    // fmt::print("vertex {} , edges {}\n", g2o::SparseOptimizer::_vertices.size(), g2o::SparseOptimizer::_edges.size());
    g2o::SparseOptimizer::optimize(n);
}

void BAoptimizer::DumpCalculationGraph(const std::string &file_name) {
    // 只保存了 HPP矩阵。
    solver->dump(file_name);
}

cv::Mat BAoptimizer::eval(const KeyFrame &kf) {
    assert(KF2ID.count(kf) != 0);
    g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(vertex(KF2ID[kf]));
    g2o::SE3Quat SE3quat = vSE3->estimate();
    return Converter::toCvMat(SE3quat);
}

cv::Mat BAoptimizer::eval(const MapPointRef &mp) {
    assert(MP2ID.count(mp) != 0);
    g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(vertex(MP2ID[mp]));
    return Converter::toCvMat(vPoint->estimate());
}

void BAoptimizer::recovery_all() {
    {
        for (KeyFrame kf : ALLKFS) {
            kf->SetPose(eval(kf));
        }
        for (MapPointRef mp : ALLMPS) {
            mp->SetWorldPose(eval(mp));
        }
    }
}

}  // namespace MCVSLAM