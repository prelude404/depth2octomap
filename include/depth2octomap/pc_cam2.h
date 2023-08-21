#ifndef PC_CAM2_H
#define PC_CAM2_H


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include <string>
#include <iostream>
#include <vector>
#include <time.h>

// Eigen 库
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

// TF 库
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
 
// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
// #include <pcl/concatenate.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

class camera
{
public:
    
    camera(int value){
        cam_num = std::to_string(value);
        listener = nullptr;
        cam_pc.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        base_pc.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }
    
    camera(int value, tf::TransformListener *lis){
        cam_num = std::to_string(value);
        // tf::TransformListener *listener = lis; // 若在构造函数内定义listener则其作用域在该函数内
        listener = lis;
        cam_pc.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        base_pc.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    std::string cam_num; // 相机编号
    double cx,cy,fx,fy; // 相机内参
    cv_bridge::CvImagePtr color_ptr, depth_ptr; // 彩色和深度图像
    cv::Mat color_pic, depth_pic; // 彩色和深度图像
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cam_pc,base_pc; // 相机和基坐标系点云
    Eigen::Matrix4d cam_to_base; // 储存cam2base
    void pic2cloud(); // 得到cam_pc, base_pc and cam_trans
    void cam2base(); // 相机坐标系转基坐标系
    double z_far_lim;
    int pc_num_lim;
    double grid_size;

private:
    double camera_factor = 1000;
    tf::TransformListener* listener; // 读取base2cam
    tf::StampedTransform cam_trans; // 储存cam2base
};


class pc_proc
{
public:
    pc_proc(tf::TransformListener *lis1, tf::TransformListener *lis2, tf::TransformListener *lis3,
            tf::TransformListener *lis4, tf::TransformListener *lis5, tf::TransformListener *lis6,
            tf::TransformListener *lis_12, tf::TransformListener *lis_21)
    {        
        // 设置ICP参数
        icp.setMaxCorrespondenceDistance(0.05);
        icp.setMaximumIterations(1000);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-6);

        listener1 = lis1;
        listener2 = lis2;
        listener3 = lis3;
        listener4 = lis4;
        listener5 = lis5;
        listener6 = lis6;
        listener_12 = lis_12;
        listener_21 = lis_21;

        merged_pc.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }
    
    pc_proc()
    {        
        // 设置ICP参数
        icp.setMaxCorrespondenceDistance(0.05);
        icp.setMaximumIterations(1000);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-6);

        listener1 = nullptr;
        listener2 = nullptr;
        listener3 = nullptr;
        listener4 = nullptr;
        listener5 = nullptr;
        listener6 = nullptr;
        listener_12 = nullptr;
        listener_21 = nullptr;

        merged_pc.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }
    
    Eigen::Matrix4d cam1_base,cam2_base,cam1_cam2,cam2_cam1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_pc;

    void icp_algo(camera& cam1, camera& cam2);

    // void g2o();
    
    void merge_cloud(camera& cam1, camera& cam2);

    // 安全判断标志
    double slow_dis1, slow_dis2; // 一级二级减速距离
    int danger_num1, danger_num2; // 各级减速点云数量
    int safe_status;

    // 圆柱增长因子
    double long_factor;

    // 圆柱包络半径
    double tlr1, tlr2, tlr3, tlr4, tlr5, tlr6;

private:
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    void cull_self(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &full_pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cull_pc);
    tf::TransformListener* listener1;
    tf::TransformListener* listener2;
    tf::TransformListener* listener3;
    tf::TransformListener* listener4;
    tf::TransformListener* listener5;
    tf::TransformListener* listener6;
    tf::TransformListener* listener_12;
    tf::TransformListener* listener_21;
    double DistanceOfPointToLine(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d s);
    bool AngleJudge(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c);
    tf::StampedTransform trans_12;
};


#endif