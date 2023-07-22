#include "ros/ros.h"
#include <std_srvs/Empty.h>
 
int main(int argc, char  *argv[])
{
    ros::init(argc,argv,"reset_node");
    ros::NodeHandle nh;
    //4 创建service客户端
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/octomap_server/reset");
    //5 等待服务启动
    //client.waitForExistence();
    ros::service::waitForService("/octomap_server/reset");
    // 6 发送请求
    while(ros::ok()){
        std_srvs::Empty empty;

        bool flag =  client.call(empty);
        usleep(5*1000000); //us
    // 7 处理响应
    }
   

    return 0;
}