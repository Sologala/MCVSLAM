#ifndef BAOPTIMIZER_H
#define BAOPTIMIZER_H
#include <g2o/core/optimizable_graph.h>
#include <g2o/core/solver.h>

#include <algorithm>

#include "Map.hpp"
#include "Object.hpp"
#pragma once

#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include <cstddef>

#include "Frame.hpp"
#include "MapPoint.hpp"
#include "OptimizeTypes.hpp"
// #include "OptimizeTypes.hpp"
namespace MCVSLAM {
class BAoptimizer : public g2o::SparseOptimizer {
   public:
    BAoptimizer();
    ~BAoptimizer();

    void addKeyFrame(const KeyFrame &kf, bool fixed = false);
    void addMapppint(const MapPointRef &mp, bool fixed = false);

    void addEdge(const KeyFrame &kf, const MapPointRef &mp, const cv::Point2f &uv, const float &level);

    void addEdgeStereo(const KeyFrame &kf, const MapPointRef &mp, const cv::Point2f &uv, float ru, const float &level);

    void addEdgeToBody(const KeyFrame &kf, const MapPointRef &mp, const cv::Point2f &uv, const float &level);

    int getEdgeSize() { return g2o::SparseOptimizer::_edges.size(); }

    int getVertexSize();

    void optimize(const int n);

    void DumpHessian(const std::string &file_name);

    cv::Mat eval(const KeyFrame &kf);
    cv::Mat eval(const MapPointRef &mp);

    // Set KeyFrame vertices
    size_t _vettex_id = 0;
    std::unordered_set<KeyFrame> ALLKFS;
    std::unordered_set<MapPointRef> ALLMPS;

    std::unordered_map<KeyFrame, size_t> KF2ID;
    std::unordered_map<MapPointRef, size_t> MP2ID;

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // filter out Edge

    typedef std::pair<g2o::OptimizableGraph::Edge *, std::pair<MapPointRef, ObjectRef>> EdgeInfoMation;
    std::unordered_map<g2o::OptimizableGraph::Edge *, std::pair<MapPointRef, ObjectRef>> EdgeInfos;

    template <class FilterCallBack>
    void filter(FilterCallBack &&callback) {
        std::for_each(EdgeInfos.begin(), EdgeInfos.end(), callback);
    }

    void recovery_all();

   private:
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    g2o::OptimizationAlgorithmLevenberg *solver;
};
}  // namespace MCVSLAM
#endif