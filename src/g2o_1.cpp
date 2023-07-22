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

class SlidingWindow {
public:
  struct PoseData {
    Eigen::Affine3d transform;
  };

  SlidingWindow(int windowSize) : windowSize_(windowSize) {}

  void addConstraint(const PoseData& poseData) {
    if (constraints_.size() >= windowSize_) {
      // 移除最早的约束
      constraints_.erase(constraints_.begin());
    }
    constraints_.push_back(poseData);
  }

  int getWindowSize() const {
    return windowSize_;
  }

  const std::vector<PoseData>& getConstraints() const {
    return constraints_;
  }

private:
  int windowSize_;
  std::vector<PoseData> constraints_;
};

int main() {
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

  int windowSize = 3;  // 滑窗长度

  // 构建图优化器
  g2o::SparseOptimizer optimizer;
  std::unique_ptr<LinearSolverType> linearSolver(new LinearSolverType());
  std::unique_ptr<BlockSolverType> blockSolver(new BlockSolverType(std::move(linearSolver)));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
  optimizer.setAlgorithm(solver);

  // 添加顶点
  std::vector<VertexSE3*> vertices;
  for (int i = 0; i < windowSize; ++i) {
    VertexSE3* v = new VertexSE3();
    v->setId(i);
    v->setEstimate(Eigen::Isometry3d::Identity());
    if (i == 0) {
      v->setFixed(true);  // 固定第一个顶点
    }
    optimizer.addVertex(v);
    vertices.push_back(v);
  }

  // 添加边
  std::vector<EdgeSE3*> edges;
  SlidingWindow slidingWindow(windowSize);  // 创建滑窗对象

  // 添加约束数据
  std::vector<SlidingWindow::PoseData> poseData = {
    { /* transform1 */ },
    { /* transform2 */ },
    { /* transform3 */ }
  };

  for (int i = 0; i < poseData.size(); ++i) {
    // 添加约束数据到滑窗
    slidingWindow.addConstraint(poseData[i]);

    if (i >= 1) {
      // 添加边到优化器
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

  // 进行优化
  optimizer.initializeOptimization();
  optimizer.optimize(10);  // 进行10次迭代

  // 输出优化后的结果
  std::cout << "Optimized transforms:" << std::endl;
  for (auto v : vertices) {
    std::cout << "Vertex " << v->id() << ":" << std::endl;
    std::cout << v->estimate().matrix() << std::endl;
  }

  return 0;
}
