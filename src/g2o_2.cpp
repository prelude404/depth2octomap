#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam3d/se3quat.h>

// 定义顶点类型
typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
typedef g2o::OptimizationAlgorithmLevenberg OptimizationAlgorithmType;
typedef g2o::SparseOptimizer OptimizerType;

class VertexSE3 : public g2o::BaseVertex<6, Eigen::Isometry3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual void setToOriginImpl() {
    _estimate = Eigen::Isometry3d::Identity();
  }

  virtual void oplusImpl(const double* update) {
    Eigen::Matrix<double, 6, 1> update_vec;
    update_vec << update[0], update[1], update[2], update[3], update[4], update[5];
    Eigen::Isometry3d increment = g2o::internal::fromVectorMQT(update_vec);
    _estimate = increment * _estimate;
  }

  virtual bool read(std::istream& /*is*/) { return false; }
  virtual bool write(std::ostream& /*os*/) const { return false; }
};

class EdgeSE3 : public g2o::BaseBinaryEdge<6, Eigen::Isometry3d, VertexSE3, VertexSE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual void computeError() {
    const VertexSE3* v1 = static_cast<const VertexSE3*>(_vertices[0]);
    const VertexSE3* v2 = static_cast<const VertexSE3*>(_vertices[1]);
    Eigen::Isometry3d relative_measurement = _inverseMeasurement * (v1->estimate().inverse() * v2->estimate());
    _error = g2o::internal::toVectorMQT(relative_measurement);
  }

  virtual bool read(std::istream& /*is*/) { return false; }
  virtual bool write(std::ostream& /*os*/) const { return false; }
};

class GraphOptimizer {
public:
  GraphOptimizer() {
    // 创建优化器
    linearSolver_ = new LinearSolverType();
    blockSolver_ = new BlockSolverType(std::unique_ptr<LinearSolverType>(linearSolver_));
    optimizationAlgorithm_ = new OptimizationAlgorithmType(std::unique_ptr<BlockSolverType>(blockSolver_));
    optimizer_.setAlgorithm(optimizationAlgorithm_);
  }

  ~GraphOptimizer() {
    // 释放内存
    delete optimizationAlgorithm_;
    delete blockSolver_;
    delete linearSolver_;
  }

  void addVertex(VertexSE3* vertex) {
    optimizer_.addVertex(vertex);
  }

  void addEdge(EdgeSE3* edge) {
    optimizer_.addEdge(edge);
  }

  void optimize(int numIterations) {
    optimizer_.initializeOptimization();
    optimizer_.optimize(numIterations);
  }

  void clear() {
    optimizer_.clear();
  }

private:
  OptimizerType optimizer_;
  BlockSolverType* blockSolver_;
  LinearSolverType* linearSolver_;
  OptimizationAlgorithmType* optimizationAlgorithm_;
};

class PoseData {
public:
  Eigen::Affine3d transform;
  std::string edgeType;
};

class SlidingWindow {
public:
  SlidingWindow(int windowSize) : windowSize_(windowSize) {}

  void addConstraint(const PoseData& poseData) {
    if (constraints_.size() >= windowSize_) {
      constraints_.erase(constraints_.begin());
    }
    constraints_.push_back(poseData);
  }

  const std::vector<PoseData>& getConstraints() const {
    return constraints_;
  }

private:
  int windowSize_;
  std::vector<PoseData> constraints_;
};

int main() {
  int windowSize = 3;

  GraphOptimizer optimizer;

  std::vector<VertexSE3*> vertices;
  for (int i = 0; i < windowSize; ++i) {
    VertexSE3* v = new VertexSE3();
    v->setId(i);
    v->setEstimate(Eigen::Isometry3d::Identity());
    if (i == 0) {
      v->setFixed(true);
    }
    optimizer.addVertex(v);
    vertices.push_back(v);
  }

  std::vector<EdgeSE3*> edges;
  SlidingWindow slidingWindow(windowSize);

  std::vector<PoseData> poseData = {
    { /* transform1 */ },
    { /* transform2 */ },
    { /* transform3 */ }
  };

  for (int i = 0; i < poseData.size(); ++i) {
    slidingWindow.addConstraint(poseData[i]);

    if (i >= 1) {
      EdgeSE3* e = new EdgeSE3();
      e->setId(i - 1);
      e->setMeasurement(poseData[i].transform.inverse() * poseData[i - 1].transform);
      e->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
      e->setVertex(0, vertices[i - 1]);
      e->setVertex(1, vertices[i]);
      optimizer.addEdge(e);
      edges.push_back(e);
    }
  }

  optimizer.optimize(10);

  std::cout << "Optimized transforms:" << std::endl;