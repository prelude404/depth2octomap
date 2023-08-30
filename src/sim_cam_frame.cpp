#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_cam_frame");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10.0);
    
    tf::TransformBroadcaster tf_broadcaster;

    Eigen::Matrix4d cam1_to_base, cam2_to_base;
    tf::StampedTransform cam1_trans, cam2_trans;
    
    // /*** CAM_2在CAM_1右侧40.0cm***/
    // cam1_to_base << 0, 0, 1, 0,
    //                -1, 0, 0, 0.20,
    //                 0,-1, 0, 0,
    //                 0, 0, 0, 1;
    // 
    // cam2_to_base << 0, 0, 1, 0,
    //                -1, 0, 0,-0.20,
    //                 0,-1, 0, 0,
    //                 0, 0, 0, 1;

    /*** CAM_2在CAM_1右侧40.0cm，朝向真实机械臂***/
    cam1_to_base << 0, 0,-1, 0,
                    1, 0, 0,-0.20,
                    0,-1, 0, 0.10,
                    0, 0, 0, 1;
    
    cam2_to_base << 0, 0,-1, 0,
                    1, 0, 0, 0.20,
                    0,-1, 0, 0.10,
                    0, 0, 0, 1;

    Eigen::Vector3d cam1_pos = cam1_to_base.block<3,1>(0,3);
    Eigen::Quaterniond cam1_quat = Eigen::Quaterniond(cam1_to_base.block<3,3>(0,0));
    cam1_trans.setOrigin(tf::Vector3(cam1_pos(0),cam1_pos(1),cam1_pos(2)));
    cam1_trans.setRotation(tf::Quaternion(cam1_quat.x(),cam1_quat.y(),cam1_quat.z(),cam1_quat.w()));

    Eigen::Vector3d cam2_pos = cam2_to_base.block<3,1>(0,3);
    Eigen::Quaterniond cam2_quat = Eigen::Quaterniond(cam2_to_base.block<3,3>(0,0));
    cam2_trans.setOrigin(tf::Vector3(cam2_pos(0),cam2_pos(1),cam2_pos(2)));
    cam2_trans.setRotation(tf::Quaternion(cam2_quat.x(),cam2_quat.y(),cam2_quat.z(),cam2_quat.w()));


    while (ros::ok())
    {
        tf_broadcaster.sendTransform(tf::StampedTransform(cam1_trans,ros::Time::now(),"base_link", "cam_1_link"));
        tf_broadcaster.sendTransform(tf::StampedTransform(cam2_trans,ros::Time::now(),"base_link", "cam_2_link"));

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
