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
        tf::TransformListener* listener = new(tf::TransformListener);
    }
    std::string cam_num; // 相机编号
    double cx,cy,fx,fy; // 相机内参
    cv_bridge::CvImagePtr color_ptr, depth_ptr; // 彩色和深度图像
    cv::Mat color_pic, depth_pic; // 彩色和深度图像
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cam_pc,base_pc; // 相机和基坐标系点云
    Eigen::Matrix4d cam_to_base; // 储存cam2base
    void pic2cloud(); // 得到cam_pc, base_pc and cam_trans
    void cam2base(); // 相机坐标系转基坐标系
private:
    double camera_factor = 1000;
    tf::TransformListener* listener; // 读取base2cam
    tf::StampedTransform cam_trans; // 储存cam2base
};

/***  彩色+深度图像转化为点云  ***/
void camera::pic2cloud()
{
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
            cam_pc->points.push_back( p );
        }
    }

    try{
        listener->waitForTransform("/base_link", ("/camera_link"+cam_num),ros::Time(0.0),ros::Duration(1.0));
        listener->lookupTransform("/base_link", ("/camera_link"+cam_num),ros::Time(0.0),cam_trans);
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
    
    // pcl::transformPointCloud(*cam_pc, *base_pc, cam_to_base); // 不确定是base_to_cam还是cam_to_base

    // 还需要运行时间和点云数量检测
    // 随机降采样
    // 直通滤波 + 体素滤波

    cam_pc->height = 1;
    cam_pc->width = cam_pc->points.size();
    cam_pc->is_dense = false;
}

class pc_proc
{
public:
    pc_proc()
    {        
        // 设置ICP参数
        icp.setMaxCorrespondenceDistance(0.05);
        icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-6);

        tf::TransformListener* listener1 = new(tf::TransformListener);
        tf::TransformListener* listener2 = new(tf::TransformListener);
        tf::TransformListener* listener3 = new(tf::TransformListener);
        tf::TransformListener* listener4 = new(tf::TransformListener);
        tf::TransformListener* listener5 = new(tf::TransformListener);
        tf::TransformListener* listener6 = new(tf::TransformListener);
    }
    
    Eigen::Matrix4d cam1_base,cam2_base,cam1_cam2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_pc;

    void icp_algo(camera& cam1, camera& cam2);

    // void g2o();
    
    void merge_cloud(camera& cam1, camera& cam2);
    void filt_cloud();

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
    double DistanceOfPointToLine(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d s);
    bool AngleJudge(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c);
};

void pc_proc::icp_algo(camera& cam1, camera& cam2)
{
    // 设置输入、输出和配准点云
    icp.setInputSource(cam1.base_pc);
    icp.setInputTarget(cam2.base_pc);
    cam1_base = cam1.cam_to_base;
    cam2_base = cam2.cam_to_base;
    
    cam1_cam2 = cam1_base * cam2_base.inverse();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_pc;

    icp.align(*aligned_pc,cam1_cam2.cast<float>()); // 提供初始猜测
    // icp.align(*aligned_pc);

    // 输出ICP配准结果
    if (icp.hasConverged())
    {
        std::cout << "ICP converged. Score: " << icp.getFitnessScore() << std::endl;
        // 获取ICP的变换矩阵
        cam1_cam2 = icp.getFinalTransformation().cast<double>();
    }
    else
    {
        std::cout << "ICP did not converge." << std::endl;
    }
}

// cam1_base,cam2_base,cam1_cam2应当利用G2O图优化进行更新

void pc_proc::merge_cloud(camera& cam1, camera& cam2)
{
    pcl::transformPointCloud(*cam1.cam_pc, *cam1.base_pc, this->cam1_base);
    pcl::transformPointCloud(*cam2.cam_pc, *cam2.base_pc, this->cam2_base);
    
    merged_pc.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    *merged_pc = *cam1.base_pc;
    *merged_pc += *cam2.base_pc;
    
    merged_pc->height = 1;
    merged_pc->width = merged_pc->points.size();

}

void pc_proc::filt_cloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr culled_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    cull_self(merged_pc,culled_pc);

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

camera cam1(1),cam2(2);
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

    cam1.cx = 331.2972717285156;
    cam1.cx = 243.7368927001953;
    cam1.fx = 606.3751831054688;
    cam1.fy = 604.959716796875;

    cam2.cx = 315.4583435058594;
    cam2.cx = 255.28733825683594;
    cam2.fx = 608.7494506835938;
    cam2.fy = 608.6277465820312;

    pc_fuser.slow_dis1 = 0.65 * 2;
    pc_fuser.slow_dis2 = 0.65;
    pc_fuser.danger_num1 = 1000;
    pc_fuser.danger_num2 = 200;
    pc_fuser.long_factor = 0.3;
    pc_fuser.tlr1 = 0.15;
    pc_fuser.tlr2 = 0.15;
    pc_fuser.tlr3 = 0.15;
    pc_fuser.tlr4 = 0.12;
    pc_fuser.tlr5 = 0.12;
    pc_fuser.tlr6 = 0.12;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub1_color = it.subscribe(("/cam_"+cam1.cam_num+"/color/image_raw"), 1, color_cb1);
    image_transport::Subscriber sub1_depth = it.subscribe(("/cam_"+cam1.cam_num+"/depth/image_rect_raw"), 1, depth_cb1);

    image_transport::ImageTransport it1(nh);
    image_transport::Subscriber sub2_color = it1.subscribe(("/cam_"+cam2.cam_num+"/color/image_raw"), 1, color_cb2);
    image_transport::Subscriber sub2_depth = it1.subscribe(("/cam_"+cam2.cam_num+"/depth/image_rect_raw"), 1, depth_cb2);

    ros::Rate loop_rate(100.0);

    while(ros::ok())
    {
        cam1.pic2cloud();
        cam2.pic2cloud();

        pc_fuser.icp_algo(cam1, cam2);
        pc_fuser.merge_cloud(cam1,cam2);

    }
    return 0;
}



