#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// 定义自定义的G2O顶点类，表示相机位姿
class VertexCameraPose : public g2o::BaseVertex<6, Eigen::Isometry3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual void setToOriginImpl() override
  {
    _estimate = Eigen::Isometry3d::Identity();
  }

  virtual void oplusImpl(const double* update) override
  {
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    Eigen::Isometry3d increment = g2o::internal::fromVectorMQT(update_eigen);
    _estimate = _estimate * increment;
  }

  virtual bool read(std::istream& /*is*/) override
  {
    return false;
  }

  virtual bool write(std::ostream& /*os*/) const override
  {
    return false;
  }
};

// 定义自定义的G2O边类，表示相邻帧之间的约束
class EdgeCameraPose : public g2o::BaseBinaryEdge<6, Eigen::Isometry3d, VertexCameraPose, VertexCameraPose>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual void computeError() override
  {
    const VertexCameraPose* v1 = static_cast<const VertexCameraPose*>(_vertices[0]);
    const VertexCameraPose* v2 = static_cast<const VertexCameraPose*>(_vertices[1]);
    Eigen::Isometry3d delta = _inverseMeasurement * (v1->estimate().inverse() * v2->estimate());
    _error = g2o::internal::toVectorMQT(delta);
  }

  virtual void linearizeOplus() override
  {
    _jacobianOplusXi = -g2o::internal::toMatrixMQT(_inverseMeasurement * _vertices[0]->estimate().inverse());
    _jacobianOplusXj = g2o::internal::toMatrixMQT(_inverseMeasurement * _vertices[0]->estimate().inverse());
  }

  virtual bool read(std::istream& /*is*/) override
  {
    return false;
  }

  virtual bool write(std::ostream& /*os*/) const override
  {
    return false;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "graph_optimization_node");
  ros::NodeHandle nh;

  // 创建TF监听器
  tf::TransformListener listener;

  // 创建滑动窗口优化器
  g2o::SparseOptimizer optimizer;
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>());
  std::unique_ptr<g2o::BlockSolver_6_3> solver(new g2o::BlockSolver_6_3(std::move(linearSolver)));
  g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(solver));
  optimizer.setAlgorithm(optimizationAlgorithm);

  // 创建点云容器和转换矩阵容器
  std::vector<PointCloudT::Ptr> pointClouds;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> cameraPoses;

  // 定义滑动窗口大小
  const int windowSize = 5;

  // 等待获取转换关系和点云
  listener.waitForTransform("/base_link", "/camera_link1", ros::Time(0), ros::Duration(3.0));
  listener.waitForTransform("/base_link", "/camera_link2", ros::Time(0), ros::Duration(3.0));

  for (int i = 0; i < windowSize; ++i)
  {
    // 获取相机位姿的转换关系
    tf::StampedTransform transform1, transform2;
    listener.lookupTransform("/base_link", "/camera_link1", ros::Time(0), transform1);
    listener.lookupTransform("/base_link", "/camera_link2", ros::Time(0), transform2);

    // 将转换关系转换为Eigen类型的变换矩阵
    Eigen::Isometry3d base_to_camera1, base_to_camera2;
    tf::transformTFToEigen(transform1, base_to_camera1);
    tf::transformTFToEigen(transform2, base_to_camera2);

    // 创建相机1和相机2的点云
    PointCloudT::Ptr cloud1(new PointCloudT);
    PointCloudT::Ptr cloud2(new PointCloudT);

    // 从ROS消息中获取点云数据，并转换到PointCloud类型
    // 从相机1获取点云消息，存储到cloud1中
    // 从相机2获取点云消息，存储到cloud2中

    // ...

    // 将点云转换到/base_link坐标系中
    pcl::transformPointCloud(*cloud1, *cloud1, base_to_camera1);
    pcl::transformPointCloud(*cloud2, *cloud2, base_to_camera2);

    // 进行ICP配准
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(cloud1);
    icp.setInputTarget(cloud2);
    PointCloudT::Ptr alignedCloud(new PointCloudT);
    icp.align(*alignedCloud);

    // 输出ICP配准结果
    if (icp.hasConverged())
    {
      std::cout << "ICP converged. Score: " << icp.getFitnessScore() << std::endl;

      // 获取ICP的变换矩阵
      Eigen::Matrix4f transformation = icp.getFinalTransformation();
      Eigen::Isometry3d camera1_to_camera2 = base_to_camera2.inverse() * base_to_camera1 * transformation.cast<double>();

      // 存储点云和相机位姿
      pointClouds.push_back(alignedCloud);
      cameraPoses.push_back(camera1_to_camera2);
    }
    else
    {
      std::cout << "ICP did not converge." << std::endl;
    }
  }

  // 将点云和相机位姿加入图中
  for (int i = 0; i < windowSize; ++i)
  {
    // 创建相机位姿顶点
    VertexCameraPose* cameraPose = new VertexCameraPose();
    cameraPose->setId(i);
    cameraPose->setEstimate(cameraPoses[i]);
    optimizer.addVertex(cameraPose);

    // 添加相邻帧之间的约束边
    if (i > 0)
    {
      EdgeCameraPose* edge = new EdgeCameraPose();
      edge->setVertex(0, optimizer.vertex(i - 1));
      edge->setVertex(1, optimizer.vertex(i));
      edge->setMeasurement(cameraPoses[i - 1].inverse() * cameraPoses[i]);
      edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
      optimizer.addEdge(edge);
    }
  }

  // 执行图优化
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  // 输出优化后的相机位姿
  for (int i = 0; i < windowSize; ++i)
  {
    VertexCameraPose* cameraPose = static_cast<VertexCameraPose*>(optimizer.vertex(i));
    std::cout << "Camera pose " << i << ":\n" << cameraPose->estimate().matrix() << std::endl;
  }

  return 0;
}
