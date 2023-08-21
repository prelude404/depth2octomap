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

time_t start, end;

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

/***  彩色+深度图像转化为点云  ***/
void camera::pic2cloud()
{   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    start = clock();
    // ROS_INFO("Cam_%s Depth_pic--Rows:%i,Cols:%i",cam_num.c_str(), depth_pic.rows, depth_pic.cols);
    // depth_pic.rows = 480; depth_pic.cols = 640;

    for (int m = 0; m < depth_pic.rows; m++)
    {
        for (int n = 0; n < depth_pic.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            float d = depth_pic.ptr<float>(m)[n];//ushort d = depth_pic.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            pcl::PointXYZRGB p;

            // // 相机模型是垂直的
            // p.x = double(d) / camera_factor;
            // p.y = -(n - camera_cx) * p.x / camera_fx;
            // p.z = -(m - camera_cy) * p.x / camera_fy;

            // 考虑坐标系转换
            p.z = double(d) / camera_factor;
            p.x = (n - cx) * p.z / fx;
            p.y = (m - cy) * p.z / fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = color_pic.ptr<uchar>(m)[n*3];
            p.g = color_pic.ptr<uchar>(m)[n*3+1];
            p.r = color_pic.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            raw_pc->points.push_back( p );
        }
    }

    // if(listener != nullptr){ROS_INFO("Listener is NOT NULL");}

    try{
        listener->waitForTransform("base_link", ("cam_"+cam_num+"_link"),ros::Time(0.0),ros::Duration(1.0));
        listener->lookupTransform("base_link", ("cam_"+cam_num+"_link"),ros::Time(0.0),cam_trans);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("camera_link: %s",ex.what());
        ros::Duration(0.5).sleep();
        return;
    }
    Eigen::Affine3d temp;
    tf::transformTFToEigen(cam_trans, temp);
    cam_to_base = temp.matrix().cast<double>();

    // 还需要运行时间和点云数量检测
    raw_pc->height = 1;
    raw_pc->width = raw_pc->points.size();
    ROS_INFO("[%s] Raw PointCloud Size = %i ",cam_num.c_str(),raw_pc->width);
    
    // 直通滤波
    pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    pass_z.setInputCloud(raw_pc);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.0,z_far_lim);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_z_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    pass_z.filter(*pass_z_pc);

    pass_z_pc->height = 1;
    pass_z_pc->width = pass_z_pc->points.size();
    ROS_INFO("[%s] Pass_Z PointCloud Size = %i ",cam_num.c_str(),pass_z_pc->width);
    
    // // 随机采样
    // // int result = (a > b) ? b : c;
    // bool random_flag = (cam_pc->width > pc_num_lim);
    // if(random_flag)
    // {
    //     pcl::RandomSample<pcl::PointXYZRGB> rs;
    //     rs.setInputCloud(cam_pc);
    //     rs.setSample(pc_num_lim);
    //     rs.filter(*cam_pc);
        
    //     cam_pc->height = 1;
    //     cam_pc->width = cam_pc->points.size();
    //     // ROS_INFO("Random Sampled PointCloud Size = %i ",cam_pc->width);
    // }
    
    // 体素滤波
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vox_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    vox.setInputCloud(pass_z_pc);
    vox.setLeafSize(grid_size, grid_size, grid_size);
    vox.filter(*vox_pc);
    
    // ROS_INFO("PointCloud before Voxel Filtering: %i data points.",(raw_pc->width * raw_pc->height));
    vox_pc->height = 1;
    vox_pc->width = vox_pc->points.size();
    ROS_INFO("[%s] Voxel Filtered PointCloud Size = %i ",cam_num.c_str(),vox_pc->width);

    vox_pc->is_dense = false;

    // cam_pc.reset();
    cam_pc = vox_pc;

    base_pc.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cam_pc, *base_pc, cam_to_base); // cam_to_base

    end = clock();
    ROS_INFO("[%s] Total Running Time is: %f secs", cam_num.c_str(), static_cast<float>(end - start) / CLOCKS_PER_SEC);

}

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

void pc_proc::icp_algo(camera& cam1, camera& cam2)
{
    cam1_base = cam1.cam_to_base;
    cam2_base = cam2.cam_to_base;

    try{
        listener_12->waitForTransform("cam_2_link", "cam_1_link",ros::Time(0.0),ros::Duration(1.0));
        listener_12->lookupTransform("cam_2_link", "cam_1_link",ros::Time(0.0),trans_12);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("camera_link: %s",ex.what());
        ros::Duration(0.5).sleep();
        return;
    }

    Eigen::Affine3d temp;
    tf::transformTFToEigen(trans_12, temp);
    cam1_cam2 = temp.matrix().cast<double>();

    cam2_cam1 = temp.inverse().matrix().cast<double>();


    // cam1_cam2 = cam1_base * cam2_base.inverse();
    ROS_INFO("Raw TF Result--(%lf, %lf, %lf)",cam1_cam2(0,3),cam1_cam2(1,3),cam1_cam2(2,3));
    
    /*** 获取两台相机重合视野部分的点云用于ICP配准 ***/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc1_2(new pcl::PointCloud<pcl::PointXYZRGB>); // cam1的点云在cam2下表示
    pcl::transformPointCloud(*cam1.cam_pc, *pc1_2, cam1_cam2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc1_2_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); // 位于公共视野的部分

    for (const auto &p : pc1_2->points)
    {
        // p.x = (n - cx) * p.z / fx;
        // p.y = (m - cy) * p.z / fy;
        
        int n = cam2.fx * p.x / p.z + cam2.cx;
        int m = cam2.fy * p.y / p.z + cam2.cy;
        
        if (n>0 && n<640 && m>0 && m<480){
            pc1_2_filtered->points.push_back(p);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc1_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); // cam1点云位于公共视野的部分
    pcl::transformPointCloud(*pc1_2_filtered, *pc1_filtered, cam2_cam1); // 自身坐标系
    
    pc1_filtered->height = 1;
    pc1_filtered->width = pc1_filtered->points.size();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc2_1(new pcl::PointCloud<pcl::PointXYZRGB>); // cam2的点云在cam1下表示
    pcl::transformPointCloud(*cam2.cam_pc, *pc2_1, cam2_cam1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc2_1_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); // 位于公共视野的部分

    for (const auto &p : pc2_1->points)
    {
        // p.x = (n - cx) * p.z / fx;
        // p.y = (m - cy) * p.z / fy;
        
        int n = cam1.fx * p.x / p.z + cam1.cx;
        int m = cam1.fy * p.y / p.z + cam1.cy;
        
        if (n>0 && n<640 && m>0 && m<480){
            pc2_1_filtered->points.push_back(p);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc2_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); // cam1点云位于公共视野的部分
    pcl::transformPointCloud(*pc2_1_filtered, *pc2_filtered, cam1_cam2); // 自身坐标系

    pc2_filtered->height = 1;
    pc2_filtered->width = pc2_filtered->points.size();

    // // 用于验证是否正确获取了两个相机公共视野区域的点云
    // cam1.base_pc.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    // cam1.base_pc = pc1_filtered;
    // cam2.base_pc.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    // cam2.base_pc = pc2_filtered;

    // 设置输入、输出和配准点云
    // icp.setInputSource(cam1.base_pc);
    // icp.setInputTarget(cam2.base_pc);
    icp.setInputSource(pc1_filtered);
    icp.setInputTarget(pc2_filtered);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_pc(new pcl::PointCloud<pcl::PointXYZRGB>);;

    icp.align(*aligned_pc,cam1_cam2.cast<float>()); // 提供初始猜测
    // icp.align(*aligned_pc);

    // 输出ICP配准结果
    if (icp.hasConverged())
    {
        std::cout << "ICP converged. Score: " << icp.getFitnessScore() << std::endl;
        // 获取ICP的变换矩阵
        cam1_cam2 = icp.getFinalTransformation().cast<double>();
        ROS_INFO("ICP Trans Result--(%lf, %lf, %lf)",cam1_cam2(0,3),cam1_cam2(1,3),cam1_cam2(2,3));
    }
    else
    {
        std::cout << "ICP did not converge." << std::endl;
    }
}

// cam1_base,cam2_base,cam1_cam2, cam2_cam1应当利用G2O图优化进行更新

void pc_proc::merge_cloud(camera& cam1, camera& cam2)
{
    // merged_pc：根据ICP+G2O的结果拼接两个相机的点云（基坐标系）
    // cam1.base_pc.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    // cam2.base_pc.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // pcl::transformPointCloud(*cam1.cam_pc, *cam1.base_pc, this->cam1_base);
    // pcl::transformPointCloud(*cam2.cam_pc, *cam2.base_pc, this->cam2_base);
    
    merged_pc.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    *merged_pc = *cam1.base_pc;
    *merged_pc += *cam2.base_pc;
    
    merged_pc->height = 1;
    merged_pc->width = merged_pc->points.size();

    ROS_INFO("Merged PointCloud Size = %i ",merged_pc->width);

    // // culled_pc：过滤机械臂自身点云并进行安全评级
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr culled_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    // cull_self(merged_pc,culled_pc);


}

void pc_proc::cull_self(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &full_pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cull_pc)
{
    tf::StampedTransform trans1, trans2, trans3, trans4, trans5, trans6;
    Eigen::Vector3d cp1, cp2, cp3, cp4, cp5, cp6;

    listener1->waitForTransform("/base_link", "/link_1",ros::Time(0.0),ros::Duration(1.0));
    listener1->lookupTransform("/base_link", "/link_1",ros::Time(0.0),trans1);
    listener2->waitForTransform("/base_link", "/link_2",ros::Time(0.0),ros::Duration(1.0));
    listener2->lookupTransform("/base_link", "/link_2",ros::Time(0.0),trans2);
    listener3->waitForTransform("/base_link", "/link_3",ros::Time(0.0),ros::Duration(1.0));
    listener3->lookupTransform("/base_link", "/link_3",ros::Time(0.0),trans3);
    listener4->waitForTransform("/base_link", "/link_4",ros::Time(0.0),ros::Duration(1.0));
    listener4->lookupTransform("/base_link", "/link_4",ros::Time(0.0),trans4);
    listener5->waitForTransform("/base_link", "/link_5",ros::Time(0.0),ros::Duration(1.0));
    listener5->lookupTransform("/base_link", "/link_5",ros::Time(0.0),trans5);
    listener6->waitForTransform("/base_link", "/link_6",ros::Time(0.0),ros::Duration(1.0));
    listener6->lookupTransform("/base_link", "/link_6",ros::Time(0.0),trans6);

    cp1[0] = double(trans1.getOrigin().x());
    cp1[1] = double(trans1.getOrigin().y());
    cp1[2] = double(trans1.getOrigin().z());
    cp2[0] = double(trans2.getOrigin().x());
    cp2[1] = double(trans2.getOrigin().y());
    cp2[2] = double(trans2.getOrigin().z());
    cp3[0] = double(trans3.getOrigin().x());
    cp3[1] = double(trans3.getOrigin().y());
    cp3[2] = double(trans3.getOrigin().z());
    cp4[0] = double(trans4.getOrigin().x());
    cp4[1] = double(trans4.getOrigin().y());
    cp4[2] = double(trans4.getOrigin().z());
    cp5[0] = double(trans5.getOrigin().x());
    cp5[1] = double(trans5.getOrigin().y());
    cp5[2] = double(trans5.getOrigin().z());
    cp6[0] = double(trans6.getOrigin().x());
    cp6[1] = double(trans6.getOrigin().y());
    cp6[2] = double(trans6.getOrigin().z());

    Eigen::Vector3d base(0.0, 0.0, 0.0);

    int obs_count1 = 0;
    int obs_count2 = 0;

    pcl::PointCloud<pcl::PointXYZRGB>::iterator index = merged_pc->begin();
    for(size_t i=0; i<merged_pc->size(); ++i)
    {
        Eigen::Vector3d c;
        c[0] = full_pc->points[i].x;
        c[1] = full_pc->points[i].y;
        c[2] = full_pc->points[i].z;

        double dis = sqrt(pow(c[0],2.0)+pow(c[1],2)+pow(c[2],2));

        if(dis > slow_dis1)
        {
          continue;
        }

        if(dis > slow_dis2)
        {
          obs_count1++;
          continue;
        }
        
        double dis1 = DistanceOfPointToLine(base, cp1, c);
        if(dis1 < tlr1 && AngleJudge(c, base, cp1) && AngleJudge(c, cp1, base))
        {
          continue;
        }
        double dis2 = DistanceOfPointToLine(cp1, cp2, c);
        if(dis2 < tlr2 && AngleJudge(c, cp1, cp2) && AngleJudge(c, cp2, cp1))
        {
          continue;
        }
        double dis3 = DistanceOfPointToLine(cp2, cp3, c);
        if(dis3 < tlr3 && AngleJudge(c, cp2, cp3) && AngleJudge(c, cp3, cp2))
        {
          continue;
        }
        double dis4 = DistanceOfPointToLine(cp3, cp4, c);
        if(dis4 < tlr4 && AngleJudge(c, cp3, cp4) && AngleJudge(c, cp4, cp3))
        {
          continue;
        }
        double dis5 = DistanceOfPointToLine(cp4, cp5, c);
        if(dis5 < tlr5 && AngleJudge(c, cp4, cp5) && AngleJudge(c, cp5, cp4))
        {
          continue;
        }
        double dis6 = DistanceOfPointToLine(cp5, cp6, c);
        if(dis6 < tlr6 && AngleJudge(c, cp5, cp6) && AngleJudge(c, cp6, cp5))
        {
          continue;
        }

        if(dis < slow_dis2)
        {
          obs_count2++;
        }

        cull_pc->push_back(full_pc->points[i]);

    }

    cull_pc->height = 1;
    cull_pc->width = cull_pc->points.size();

    if(obs_count2 > danger_num2)
    {
        safe_status = 1;
        ROS_INFO("### 222-LEVEL SPEED DOWN ###");
    }
    else if(obs_count1 > danger_num1)
    {
        safe_status = 2;
        ROS_INFO("### 111-LEVEL SPEED DOWN ###");
    }
    else{
        safe_status = 3;
        ROS_INFO("### SAFE ###");
    }

    ROS_INFO("Slow_down 1 Num = %i ",obs_count1);
    ROS_INFO("Slow_down 2 Num (Self_Culled) = %i ",obs_count2);

    return;
}

/*** 点到直线距离 ***/
double pc_proc::DistanceOfPointToLine(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d s) 
{ 
    // std::cout<<"a:"<<b->x<<b->y<<b->z<<std::endl;
    double ab = sqrt(pow((a[0] - b[0]), 2.0) + pow((a[1] - b[1]), 2.0) + pow((a[2] - b[2]), 2.0));
    double as = sqrt(pow((a[0] - s[0]), 2.0) + pow((a[1] - s[1]), 2.0) + pow((a[2] - s[2]), 2.0));
    double bs = sqrt(pow((s[0] - b[0]), 2.0) + pow((s[1] - b[1]), 2.0) + pow((s[2] - b[2]), 2.0));
    double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab*as);
    double sin_A = sqrt(1 - pow(cos_A, 2.0));
    // std::cout<<"ab:"<<ab<<"as:"<<as<<"bs:"<<bs<<"cosA:"<<cos_A<<std::endl;
    return as*sin_A; 
}

/*** 判断<ABC为钝角(0)或锐角(1) ***/
bool pc_proc::AngleJudge(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c)
{
    double inner = (a[0] - (1+long_factor)*b[0] + long_factor*c[0])*(c[0] - b[0]) + (a[1] - (1+long_factor)*b[1] + long_factor*c[1])*(c[1] - b[1]) + (a[2] - (1+long_factor)*b[2] + long_factor*c[2])*(c[2] - b[2]);
    if(inner > 0) //锐角
    {return true;}
    else{return false;}
}

camera cam1(1), cam2(2);
pc_proc pc_fuser;

/***  CAM1 RGB处理  ***/
void color_cb1(const sensor_msgs::ImageConstPtr& color_msg)
{
    try
    {
        cam1.color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
        cv::waitKey(1050); // 不断刷新图像，频率时间为int delay，单位为ms
    }
    catch (cv_bridge::Exception& e )
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", color_msg->encoding.c_str());
    }
    cam1.color_pic = cam1.color_ptr->image;
}
 
/***  CAM1 Depth处理  ***/
void depth_cb1(const sensor_msgs::ImageConstPtr& depth_msg)
{
    try
    {
        cam1.depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::waitKey(1050);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
    }
 
    cam1.depth_pic = cam1.depth_ptr->image;
}

/***  CAM2 RGB处理  ***/
void color_cb2(const sensor_msgs::ImageConstPtr& color_msg)
{
    try
    {
        cam2.color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
        cv::waitKey(1050); // 不断刷新图像，频率时间为int delay，单位为ms
    }
    catch (cv_bridge::Exception& e )
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", color_msg->encoding.c_str());
    }
    cam2.color_pic = cam2.color_ptr->image;
}
 
/***  CAM2 Depth处理  ***/
void depth_cb2(const sensor_msgs::ImageConstPtr& depth_msg)
{
    try
    {
        cam2.depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::waitKey(1050);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
    }
 
    cam2.depth_pic = cam2.depth_ptr->image;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_cam2");
    ros::NodeHandle nh;

    tf::TransformListener* lis_cam1 = new(tf::TransformListener);
    tf::TransformListener* lis_cam2 = new(tf::TransformListener);

    cam1 = camera(1,lis_cam1);
    cam2 = camera(2,lis_cam2);

    tf::TransformListener* lis1 = new(tf::TransformListener);
    tf::TransformListener* lis2 = new(tf::TransformListener);
    tf::TransformListener* lis3 = new(tf::TransformListener);
    tf::TransformListener* lis4 = new(tf::TransformListener);
    tf::TransformListener* lis5 = new(tf::TransformListener);
    tf::TransformListener* lis6 = new(tf::TransformListener);

    tf::TransformListener* lis_12 = new(tf::TransformListener);
    tf::TransformListener* lis_21 = new(tf::TransformListener);

    pc_fuser = pc_proc(lis1, lis2, lis3, lis4, lis5, lis6, lis_12, lis_21);

    // // 采用深度相机的内参
    // cam1.fx = 428.811279296875;
    // cam1.fy = 428.811279296875;
    // cam1.cx = 428.26531982421875;
    // cam1.cy = 241.0136260986328;

    // 采用彩色相机的内参
    cam1.fx = 608.7494506835938;
    cam1.fy = 608.6277465820312;
    cam1.cx = 315.4583435058594;
    cam1.cy = 255.28733825683594;

    nh.getParam("view_field",cam1.z_far_lim);
    nh.getParam("imput_num",cam1.pc_num_lim);
    nh.getParam("grid_size",cam1.grid_size);

    // // 采用深度相机的内参
    // cam2.fx = 421.06524658203125;
    // cam2.fy = 421.06524658203125;
    // cam2.cx = 423.03704833984375;
    // cam2.cy = 231.8202362060547;

    // 采用彩色相机的内参
    cam2.fx = 606.3751831054688;
    cam2.fy = 604.959716796875;
    cam2.cx = 331.2972717285156;
    cam2.cy = 243.7368927001953;

    nh.getParam("view_field",cam2.z_far_lim);
    nh.getParam("imput_num",cam2.pc_num_lim);
    nh.getParam("grid_size",cam2.grid_size);


    nh.getParam("slow_dis1",pc_fuser.slow_dis1);
    nh.getParam("slow_dis2",pc_fuser.slow_dis2);
    nh.getParam("danger_num1",pc_fuser.danger_num1);
    nh.getParam("danger_num2",pc_fuser.danger_num2);
    nh.getParam("long_factor",pc_fuser.long_factor);
    nh.getParam("link1_radius",pc_fuser.tlr1);
    nh.getParam("link2_radius",pc_fuser.tlr2);
    nh.getParam("link3_radius",pc_fuser.tlr3);
    nh.getParam("link4_radius",pc_fuser.tlr4);
    nh.getParam("link5_radius",pc_fuser.tlr5);
    nh.getParam("link6_radius",pc_fuser.tlr6);

    image_transport::ImageTransport it1(nh);
    image_transport::Subscriber sub1_color = it1.subscribe(("/cam_"+cam1.cam_num+"/color/image_raw"), 1, color_cb1);
    // image_transport::Subscriber sub1_depth = it1.subscribe(("/cam_"+cam1.cam_num+"/depth/image_rect_raw"), 1, depth_cb1);
    image_transport::Subscriber sub1_depth = it1.subscribe(("/cam_"+cam1.cam_num+"/aligned_depth_to_color/image_raw"), 1, depth_cb1);

    image_transport::ImageTransport it2(nh);
    image_transport::Subscriber sub2_color = it2.subscribe(("/cam_"+cam2.cam_num+"/color/image_raw"), 1, color_cb2);
    // image_transport::Subscriber sub2_depth = it2.subscribe(("/cam_"+cam2.cam_num+"/depth/image_rect_raw"), 1, depth_cb2);
    image_transport::Subscriber sub2_depth = it2.subscribe(("/cam_"+cam2.cam_num+"/aligned_depth_to_color/image_raw"), 1, depth_cb2);


    // 发布合成点云和安全评级
    // ros::Publisher merge_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/merged", 1);
    ros::Publisher merge_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);
    ros::Publisher safe_status_publisher = nh.advertise<std_msgs::Int32>("/safe_status",1);

    // 发布两台相机各自的点云
    ros::Publisher pc1_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_1", 1);
    ros::Publisher pc2_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_2", 1);
    sensor_msgs::PointCloud2 pc1_msg;
    sensor_msgs::PointCloud2 pc2_msg;

    sensor_msgs::PointCloud2 merge_msg;
    std_msgs::Int32 safe_msg;
    
    ros::Rate loop_rate(10.0);

    while(ros::ok())
    {
        cam1.pic2cloud();
        cam2.pic2cloud();

        // pc_fuser.icp_algo(cam1, cam2);
        pc_fuser.merge_cloud(cam1,cam2);

        // 发布两台相机各自的点云
        // pcl::toROSMsg(*cam1.cam_pc,pc1_msg);
        // pc1_msg.header.frame_id = "cam_1_link";
        pcl::toROSMsg(*cam1.base_pc,pc1_msg);
        pc1_msg.header.frame_id = "base_link";
        pc1_msg.header.stamp = ros::Time::now();
        pc1_pub.publish(pc1_msg);
        
        // pcl::toROSMsg(*cam2.cam_pc,pc2_msg);
        // pc2_msg.header.frame_id = "cam_2_link";
        pcl::toROSMsg(*cam2.base_pc,pc2_msg);
        pc2_msg.header.frame_id = "base_link";
        pc2_msg.header.stamp = ros::Time::now();
        pc2_pub.publish(pc2_msg);

    //     // 发布安全状态（1二级减速；2一级减速；3安全）
    //     safe_msg.data = pc_fuser.safe_status;
    //     safe_status_publisher.publish(safe_msg);

        // 发布用于建图的最终结果点云
        pcl::toROSMsg(*pc_fuser.merged_pc,merge_msg);  //之后改为最终的点云融合滤波结果
        merge_msg.header.frame_id = "base_link";
        merge_msg.header.stamp = ros::Time::now();
        merge_pub.publish(merge_msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}




