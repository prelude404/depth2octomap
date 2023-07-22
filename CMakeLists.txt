cmake_minimum_required(VERSION 3.0.2)
project(depth2octomap)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rostime
  sensor_msgs
  message_filters
  cv_bridge
  image_transport
  tf
  tf_conversions
  std_srvs
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(Eigen3 REQUIRED)


catkin_package()
 
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
 
add_executable(depth2octomap src/depth2octomap.cpp)
target_link_libraries(depth2octomap ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(cam_frame_update src/cam_frame_update.cpp)
target_link_libraries(cam_frame_update ${catkin_LIBRARIES})

add_executable(pointcloud_filter src/pointcloud_filter.cpp)
target_link_libraries(pointcloud_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(pc_cam2 src/pc_cam2.cpp)
target_link_libraries(pc_cam2 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(reset_node src/reset_node.cpp)
target_link_libraries(reset_node ${catkin_LIBRARIES})
