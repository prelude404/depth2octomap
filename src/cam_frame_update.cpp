#include <iostream>
#include <ros/ros.h>
// #include <Eigen/Core>
// #include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/Eigen>
#include <tf/transform_broadcaster.h>
// #include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <math.h>

double l1=69; //mm
double l2=10.5; //mm
double l3=78; //mm
const Eigen::Vector3d fix_point(0,-500,-800); // active vision system fix point in world frame

const double pi = 3.14159265358979323846;

ros::Publisher servo1_pub;
ros::Publisher servo2_pub;


Eigen::Vector2d get_servo_angle(tf::StampedTransform end_trans)
{
    Eigen::Vector3d current_end(end_trans.getOrigin().x()*1000,end_trans.getOrigin().y()*1000,end_trans.getOrigin().z()*1000);

    Eigen::Vector2d servo_angle(0,0);
    servo_angle(0) = atan2(fix_point(0)-current_end(0),current_end(1)-fix_point(1)); // rad
    double dis = sqrt(pow(fix_point(0)-l1*sin(servo_angle(0))-current_end(0),2)+pow(fix_point(1)-l2*cos(servo_angle(0))-current_end(1),2)); //mm
    servo_angle(1) = pi/2.0 - atan2(current_end(2)-l1-fix_point(2),dis); //rad

    return servo_angle;
}

void TF_update(Eigen::Vector2d servo_angle)
{
    // The TF::TransformBroadcaster should be static!!
    static tf::TransformBroadcaster fix_broadcaster;
    static tf::TransformBroadcaster cam_broadcaster;
    
    tf::StampedTransform cam_trans;
    tf::StampedTransform fix_trans;

    // fix_link
    fix_trans.setOrigin(tf::Vector3(fix_point(0)*0.001,fix_point(1)*0.001,0));
    tf::Quaternion quat;
    quat.setRPY(0,0,0);
    fix_trans.setRotation(quat);
    fix_broadcaster.sendTransform(tf::StampedTransform(fix_trans, ros::Time::now(), "/base_link","/fix_link"));

    // camera_link
    Eigen::Matrix4d g0, e1, e2, t0;
    
    g0 << 1,0,0,0,
          0,1,0,l2,
          0,0,1,l1+l3,
          0,0,0,1;

    e1 << cos(servo_angle(0)), -sin(servo_angle(0)), 0, 0,
          sin(servo_angle(0)), cos(servo_angle(0)) , 0, 0,
          0,0,1,0,
          0,0,0,1;
    
    e2 << 1,0,0,0,
          0, cos(servo_angle(1)), sin(servo_angle(1)), l2*(1-cos(servo_angle(1)))-l1*sin(servo_angle(1)),
          0, -sin(servo_angle(1)),cos(servo_angle(1)),l2*sin(servo_angle(1))+l2*(1-cos(servo_angle(1))),
          0,0,0,1;

    // t0 << 1,0,0,fix_point(0),
    //       0,1,0,fix_point(1),
    //       0,0,1,fix_point(2),
    //       0,0,0,1;
    
    t0 << 1,0,0,0,
          0,1,0,0,
          0,0,1,fix_point(2),
          0,0,0,1;
    
    Eigen::Matrix4d cam_trans_matrix;
    cam_trans_matrix = t0 * e1 * e2 * g0;
    // cam_trans_matrix = e1 * e2 * g0;

    // std::cout << cam_trans_matrix;

    Eigen::Vector3d cam_pos = cam_trans_matrix.block<3,1>(0,3);
    Eigen::Quaterniond cam_quat = Eigen::Quaterniond(cam_trans_matrix.block<3,3>(0,0));
    cam_trans.setOrigin(tf::Vector3(cam_pos(0)*0.001,cam_pos(1)*0.001,(cam_pos(2)-fix_point(2))*0.001));
    cam_trans.setRotation(tf::Quaternion(cam_quat.x(),cam_quat.y(),cam_quat.z(),cam_quat.w()));
    // cam_broadcaster.sendTransform(tf::StampedTransform(cam_trans,ros::Time::now(),"/base_link","/camera_link"));
    cam_broadcaster.sendTransform(tf::StampedTransform(cam_trans,ros::Time::now(),"/fix_link","/camera_link"));
    // ROS_INFO("updating TF TREE");
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"cam_frame_update");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1000.0);

    servo1_pub = nh.advertise<std_msgs::Float64>("servo1", 10);
    servo2_pub = nh.advertise<std_msgs::Float64>("servo2", 10);
    
    Eigen::Vector2d servo_angle(0,0);
    std_msgs::Float64 ang1,ang2;

    tf::TransformListener end_listener;
    tf::StampedTransform end_trans;

    // fix_link虽然不变但也不能只发一次，需要持续发送

    while(ros::ok())
    {
        try{
            end_listener.waitForTransform("/world", "/link_6",ros::Time(0.0),ros::Duration(1.0));
            end_listener.lookupTransform("/world", "/link_6",ros::Time(0.0),end_trans);

            servo_angle = get_servo_angle(end_trans);
        
            // ROS_INFO("Servo Angle 1:%d",servo_angle(0));
            // ROS_INFO("Servo Angle 2:%d",servo_angle(1));

            ang1.data = servo_angle(0)*180.0/pi;
            ang2.data = servo_angle(1)*180.0/pi;
            // servo1_pub.publish(ang1);
            // servo2_pub.publish(ang2);

            TF_update(servo_angle);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("link_6: %s",ex.what());
            ros::Duration(0.5).sleep();
            continue;
        }

        loop_rate.sleep();
    }
}
