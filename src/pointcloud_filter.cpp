#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
 
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

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Eigen>
 
// 定义点云类型
// typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


// camera_info话题读取的相机内参
const double camera_factor = 1000;
const double camera_cx = 331.2972717285156;
const double camera_cy = 243.7368927001953;
const double camera_fx = 606.3751831054688;
const double camera_fy = 604.959716796875;
 
// 全局变量：图像矩阵和点云
cv_bridge::CvImagePtr color_ptr, depth_ptr;
cv::Mat color_pic, depth_pic;


// 三维坐标点
struct cPoint
{
	double x;
	double y;
	double z;
};

cPoint cp1;
cPoint cp2;
cPoint cp3;
cPoint cp4;
cPoint cp5;
cPoint cp6;

void color_Callback(const sensor_msgs::ImageConstPtr& color_msg);
void depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg);
void pic2cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
void cam2base(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cam_pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &base_pc);
void cull_self(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &full_pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cull_pc, int& obs_count1, int& obs_count2);


// 关节状态获取
/* TF listener用到了ROS nodehandle句柄，因此在init+nodehandle之前不能使用
   若要定义为全局变量，需要先定义为空指针再在main()的nodehandle之后初始化
   之后对于listener的使用就不能用“.”而得用“->” */
tf::TransformListener* listener1 = NULL;
tf::TransformListener* listener2 = NULL;
tf::TransformListener* listener3 = NULL;
tf::TransformListener* listener4 = NULL;
tf::TransformListener* listener5 = NULL;
tf::TransformListener* listener6 = NULL;
tf::StampedTransform trans1;
tf::StampedTransform trans2;
tf::StampedTransform trans3;
tf::StampedTransform trans4;
tf::StampedTransform trans5;
tf::StampedTransform trans6;

// 安全判断标志
double slow_dis1 = 0.65+0.6; // 一级减速距离
double slow_dis2 = 0.65; // 二级减速距离
int danger_num1 = 1000;
int danger_num2 = 200;

// 圆柱增长因子
double long_factor = 0.3;

// 圆柱包络半径
double tlr1 = 0.15;
double tlr2 = 0.15;
double tlr3 = 0.15;
double tlr4 = 0.12;
double tlr5 = 0.12;
double tlr6 = 0.12;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_octomap");
  ros::NodeHandle nh;

  listener1 = new(tf::TransformListener);
  listener2 = new(tf::TransformListener);
  listener3 = new(tf::TransformListener);
  listener4 = new(tf::TransformListener);
  listener5 = new(tf::TransformListener);
  listener6 = new(tf::TransformListener);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, color_Callback);
  image_transport::Subscriber sub1 = it.subscribe("/camera/depth/image_rect_raw", 1, depth_Callback);
  ros::Publisher pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/filter", 1);
  
  ros::Publisher safe_status_publisher = nh.advertise<std_msgs::Int32>("/safe_status",1);
  std_msgs::Int32 safe_msg;
  int obs_num1; // 一级减速距离之内的点云数量
  int obs_num2; // 二级减速距离之内的点云数量

  // 点云变量
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_pc(new pcl::PointCloud<pcl::PointXYZRGB>); // 原始有色点云
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr vox_pc(new pcl::PointCloud<pcl::PointXYZRGB>); // 体素滤波后的点云
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_pc_z(new pcl::PointCloud<pcl::PointXYZRGB>); // 相机z轴远处过滤后的点云
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_pc_x(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_pc_y(new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr noNaN_pc(new pcl::PointCloud<pcl::PointXYZRGB>); // 之后会直通滤波
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_pc(new pcl::PointCloud<pcl::PointXYZRGB>); // 世界坐标系的点云
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_pc1(new pcl::PointCloud<pcl::PointXYZRGB>); // 过滤桌面后的点云
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_pc2(new pcl::PointCloud<pcl::PointXYZRGB>); // 直通滤波后的点云
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_pc3(new pcl::PointCloud<pcl::PointXYZRGB>); // 直通滤波后的点云
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cull_pc(new pcl::PointCloud<pcl::PointXYZRGB>); // 剔除自身机械臂后的点云

  sensor_msgs::PointCloud2 pub_pointcloud;

  ros::Rate loop_rate(100.0); // use to regulate loop rate

  while (ros::ok()) {
      
      // 图像转化为点云
      pic2cloud(raw_pc);

      // 设置点云
      raw_pc->height = 1;
      // 避免报错：Assertion `cloud.points.size () == cloud.width * cloud.height' failed.
      raw_pc->width = raw_pc->points.size();
      // raw_pc->width = raw_pc->width * raw_pc->height;
      ROS_INFO("Raw PointCloud Size = %i",int(raw_pc->points.size()));
      raw_pc->is_dense = false;

      /* [pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.
        由于原始点云的范围过大，利用体素滤波划分0.02m的方格时可能使叶节点索引数量过多
        所以应先对相机坐标系下点云Z轴方向进行直通滤波 */

      // // 过滤相机z轴约arm臂展三倍距离外的点云，单位m
      double arm_len = 0.65 * 2;
      pcl::PassThrough<pcl::PointXYZRGB> pass_z;
      // pass_z.setInputCloud(vox_pc);
      pass_z.setInputCloud(raw_pc);
      pass_z.setFilterFieldName("z");
      pass_z.setFilterLimits(0.0,arm_len);
      pass_z.filter(*pass_pc_z);

      pass_pc_z->height = 1;
      pass_pc_z->width = pass_pc_z->points.size();
      ROS_INFO("Passed Cam_Z PointCloud Size = %i ",pass_pc_z->width);


      // // 体素滤波依然会报叶节点过小的错，因此把x和y轴也过滤，但没有减少点，故不采用
      // pcl::PassThrough<pcl::PointXYZRGB> pass_x;
      // pass_x.setInputCloud(pass_pc_z);
      // pass_x.setFilterFieldName("x");
      // pass_x.setFilterLimits(-arm_len/2.0,arm_len/2.0);
      // pass_x.filter(*pass_pc_x);

      // pcl::PassThrough<pcl::PointXYZRGB> pass_y;
      // pass_y.setInputCloud(pass_pc_x);
      // pass_y.setFilterFieldName("y");
      // pass_y.setFilterLimits(arm_len/2.0,arm_len/2.0);
      // pass_y.filter(*pass_pc_y);
      // ROS_INFO("Passed Cam_XYZ PointCloud Size = %i ",(pass_pc_z->width * pass_pc_z->height));

      
      // 体素滤波
      pcl::VoxelGrid<pcl::PointXYZRGB> vox;
      vox.setInputCloud(pass_pc_z);            // 输入点云
      vox.setLeafSize(0.01f, 0.01f, 0.01f); // 体素滤波器，单位m
      vox.filter(*vox_pc);                  // 体素滤波后的点云
      
      // ROS_INFO("PointCloud before Voxel Filtering: %i data points.",(raw_pc->width * raw_pc->height));
      vox_pc->height = 1;
      vox_pc->width = vox_pc->points.size();
      ROS_INFO("Voxel Filtered PointCloud Size = %i ",vox_pc->width);

            

      // 坐标系转换：/camera_link to /base_link
      // cam2base(pass_pc_y,base_pc);
      // ROS_INFO("PointCloud after Frame Trans: %i data points.",(base_pc->width * base_pc->height));
      cam2base(vox_pc,base_pc);

      base_pc->height = 1;
      base_pc->width = base_pc->points.size();

      // 直通滤波过滤桌面及以下点云
      pcl::PassThrough<pcl::PointXYZRGB> pass1;
      pass1.setInputCloud(base_pc);
      pass1.setFilterFieldName("z");
      pass1.setFilterLimits(0.1,arm_len);
      pass1.filter(*pass_pc1);

      pass_pc1->height = 1;
      pass_pc1->width = pass_pc1->points.size();
      ROS_INFO("Passed Table PointCloud Size = %i ",pass_pc1->width);
      // ROS_INFO("Passed PointCloud Size2 = %i ",int(pass_pc1->size())); //点云size为0时会报错

      // 机械臂自身点云剔除
      cull_self(pass_pc1, cull_pc, obs_num1, obs_num2);

      cull_pc->height = 1;
      cull_pc->width = cull_pc->points.size();
      // ROS_INFO("Self Culled PointCloud Size = %i ",cull_pc->width);

      // pcl::toROSMsg(*cull_pc,pub_pointcloud);
      pcl::toROSMsg(*cull_pc,pub_pointcloud); // ***
      // pcl::toROSMsg(*pass_pc3,pub_pointcloud);
      pub_pointcloud.header.frame_id = "base_link";
      pub_pointcloud.header.stamp = ros::Time::now();
      // 发布合成点云
      pointcloud_publisher.publish(pub_pointcloud);

      // 发布安全状态
      if(obs_num2 > danger_num2)
      {
        safe_msg.data = 1;
        ROS_INFO("### 222-LEVEL SPEED DOWN ###");
      }
      else if(obs_num1 > danger_num1)
      {
        safe_msg.data = 2;
        ROS_INFO("### 111-LEVEL SPEED DOWN ###");
      }
      else{
        safe_msg.data = 3;
        ROS_INFO("### SAFE ###");
      }
      
      safe_status_publisher.publish(safe_msg);

      ROS_INFO("Slow_down 1 Num = %i ",obs_num1);
      ROS_INFO("Slow_down 2 Num (Self_Culled) = %i ",obs_num2);

      // 清除数据并退出
      raw_pc->points.clear();
      vox_pc->points.clear();
      pass_pc_z->points.clear();
      pass_pc_x->points.clear();
      pass_pc_y->points.clear();
      base_pc->points.clear();
      pass_pc1->points.clear();
      cull_pc->points.clear();
      pub_pointcloud.data.clear();  // ***

      ros::spinOnce(); // call all of the awaiting callback() functions
      loop_rate.sleep(); // wait for remainder of specified period;
  }
}


/***  RGB处理  ***/
void color_Callback(const sensor_msgs::ImageConstPtr& color_msg)
{
    try
    {
        color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
        cv::waitKey(1050); // 不断刷新图像，频率时间为int delay，单位为ms
    }
    catch (cv_bridge::Exception& e )
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", color_msg->encoding.c_str());
    }
    color_pic = color_ptr->image;
}
 
/***  Depth处理  ***/
void depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    try
    {
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::waitKey(1050);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
    }
 
    depth_pic = depth_ptr->image;
}

/*** 点云读取 ***/
void pic2cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
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
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = color_pic.ptr<uchar>(m)[n*3];
            p.g = color_pic.ptr<uchar>(m)[n*3+1];
            p.r = color_pic.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    }
}

/*** 坐标转换 ***/
void cam2base(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cam_pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &base_pc)
{
    tf::TransformListener cam_listener; // not sure should be static or not
    tf::StampedTransform cam_trans;

    Eigen::Matrix4d cam_trans_mat;

    try{
        cam_listener.waitForTransform("/base_link", "/camera_link",ros::Time(0.0),ros::Duration(1.0));
        cam_listener.lookupTransform("/base_link", "/camera_link",ros::Time(0.0),cam_trans);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("camera_link: %s",ex.what());
        ros::Duration(0.5).sleep();
        return;
    }

    cam_trans_mat.block<3,1>(0,3) << cam_trans.getOrigin().getX(), cam_trans.getOrigin().getY(), cam_trans.getOrigin().getZ();
    
    Eigen::Quaterniond eigen_cam_quat(cam_trans.getRotation().getW(),cam_trans.getRotation().getX(),cam_trans.getRotation().getY(),cam_trans.getRotation().getZ());
    
    cam_trans_mat.block<3,3>(0,0) << eigen_cam_quat.toRotationMatrix();

    // Eigen::Translation3d tl_btol(cam_trans.getOrigin().getX(), cam_trans.getOrigin().getY(), cam_trans.getOrigin().getZ());
    // double roll, pitch, yaw;
    // tf::Matrix3x3(cam_trans.getRotation()).getEulerYPR(yaw, pitch, roll);
    // Eigen::AngleAxisd rot_x_btol(roll, Eigen::Vector3d::UnitX());
    // Eigen::AngleAxisd rot_y_btol(pitch, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd rot_z_btol(yaw, Eigen::Vector3d::UnitZ());
    // cam_trans_mat = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
    pcl::transformPointCloud(*cam_pc, *base_pc, cam_trans_mat);

}

/*** 点到直线距离 ***/
double DistanceOfPointToLine(cPoint* a, cPoint* b, cPoint* s) 
{ 
  // std::cout<<"a:"<<b->x<<b->y<<b->z<<std::endl;
	double ab = sqrt(pow((a->x - b->x), 2.0) + pow((a->y - b->y), 2.0) + pow((a->z - b->z), 2.0));
	double as = sqrt(pow((a->x - s->x), 2.0) + pow((a->y - s->y), 2.0) + pow((a->z - s->z), 2.0));
	double bs = sqrt(pow((s->x - b->x), 2.0) + pow((s->y - b->y), 2.0) + pow((s->z - b->z), 2.0));
	double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab*as);
	double sin_A = sqrt(1 - pow(cos_A, 2.0));
  // std::cout<<"ab:"<<ab<<"as:"<<as<<"bs:"<<bs<<"cosA:"<<cos_A<<std::endl;
	return as*sin_A; 
}

/*** 判断<ABC为钝角(0)或锐角(1) ***/
bool AngleJudge(cPoint* a, cPoint* b, cPoint* c)
{
  double inner = (a->x - (1+long_factor)*b->x + long_factor*c->x)*(c->x - b->x) + (a->y - (1+long_factor)*b->y + long_factor*c->y)*(c->y - b->y) + (a->z - (1+long_factor)*b->z + long_factor*c->z)*(c->z - b->z);
  if(inner > 0) //锐角
  {return true;}
  else{return false;}
}

void LinkPosition(cPoint& cp1, cPoint& cp2, cPoint& cp3, cPoint& cp4,cPoint& cp5, cPoint& cp6,
                  tf::StampedTransform& trans1, tf::StampedTransform& trans2, tf::StampedTransform& trans3, 
                  tf::StampedTransform& trans4, tf::StampedTransform& trans5, tf::StampedTransform& trans6)
{
    cp1.x = double(trans1.getOrigin().x());
    cp1.y = double(trans1.getOrigin().y());
    cp1.z = double(trans1.getOrigin().z());
    cp2.x = double(trans2.getOrigin().x());
    cp2.y = double(trans2.getOrigin().y());
    cp2.z = double(trans2.getOrigin().z());
    cp3.x = double(trans3.getOrigin().x());
    cp3.y = double(trans3.getOrigin().y());
    cp3.z = double(trans3.getOrigin().z());
    cp4.x = double(trans4.getOrigin().x());
    cp4.y = double(trans4.getOrigin().y());
    cp4.z = double(trans4.getOrigin().z());
    cp5.x = double(trans5.getOrigin().x());
    cp5.y = double(trans5.getOrigin().y());
    cp5.z = double(trans5.getOrigin().z());
    cp6.x = double(trans6.getOrigin().x());
    cp6.y = double(trans6.getOrigin().y());
    cp6.z = double(trans6.getOrigin().z());
    
    return;
}

/*** 机械臂自身点云剔除 ***/
/*** 顺便统计危险区域障碍物点数量 ***/
void cull_self(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &full_pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cull_pc, int& obs_count1, int& obs_count2)
{
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

    LinkPosition(cp1,cp2,cp3,cp4,cp5,cp6,trans1,trans2,trans3,trans4,trans5,trans6);

    obs_count1 = 0;
    obs_count2 = 0;

    cPoint base;
    base.x = 0.0;
    base.y = 0.0;
    base.z = 0.0;

    // link radius tolerance
    double tlr1 = 0.15;
    double tlr2 = 0.15;
    double tlr3 = 0.15;
    double tlr4 = 0.12;
    double tlr5 = 0.12;
    double tlr6 = 0.12;

    pcl::PointCloud<pcl::PointXYZRGB>::iterator index = full_pc->begin();
    for(size_t i=0; i<full_pc->size(); ++i)
    {
      cPoint c;
      c.x = full_pc->points[i].x;
      c.y = full_pc->points[i].y;
      c.z = full_pc->points[i].z;

      double dis = sqrt(pow(c.x,2.0)+pow(c.y,2)+pow(c.z,2));

      if(dis > slow_dis1)
      {
        continue;
      }

      if(dis > slow_dis2)
      {
        obs_count1++;
        continue;
      }
      
      double dis1 = DistanceOfPointToLine(&base, &cp1, &c);
      if(dis1 < tlr1 && AngleJudge(&c, &base, &cp1) && AngleJudge(&c, &cp1, &base))
      {
        continue;
      }
      double dis2 = DistanceOfPointToLine(&cp1, &cp2, &c);
      if(dis2 < tlr2 && AngleJudge(&c, &cp1, &cp2) && AngleJudge(&c, &cp2, &cp1))
      {
        continue;
      }
      double dis3 = DistanceOfPointToLine(&cp2, &cp3, &c);
      if(dis3 < tlr3 && AngleJudge(&c, &cp2, &cp3) && AngleJudge(&c, &cp3, &cp2))
      {
        continue;
      }
      double dis4 = DistanceOfPointToLine(&cp3, &cp4, &c);
      if(dis4 < tlr4 && AngleJudge(&c, &cp3, &cp4) && AngleJudge(&c, &cp4, &cp3))
      {
        continue;
      }
      double dis5 = DistanceOfPointToLine(&cp4, &cp5, &c);
      if(dis5 < tlr5 && AngleJudge(&c, &cp4, &cp5) && AngleJudge(&c, &cp5, &cp4))
      {
        continue;
      }
      double dis6 = DistanceOfPointToLine(&cp5, &cp6, &c);
      if(dis6 < tlr6 && AngleJudge(&c, &cp5, &cp6) && AngleJudge(&c, &cp6, &cp5))
      {
        continue;
      }

      if(dis < slow_dis2)
      {
        obs_count2++;
      }

      cull_pc->push_back(full_pc->points[i]);

    }

    return;
}