cmake_minimum_required(VERSION 2.8.3)
project(TiagoBears_PoseEstimation)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  nav_msgs
  geometry_msgs
  tf
  sensor_msgs
  message_generation
  nav_msgs
  tf2
  tf2_ros 
  
)

find_package(PCL REQUIRED COMPONENTS common visualization io)

add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  FILES
  msg/cloud_clusters.msg
  #msg/PoseEstimation.msg
)

add_service_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
   FILES
   srv/PoseEstimation.srv
)


generate_messages(DEPENDENCIES sensor_msgs geometry_msgs nav_msgs) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs pcl_ros tf message_runtime tf2 tf2_ros # add some packages
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# add here header files
add_library(${PROJECT_NAME}
  src/pose_estimator.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})





# for each new node
add_executable(pose_node scripts/pose_node.cpp src/pose_estimator.cpp)
add_dependencies(pose_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_node ${catkin_LIBRARIES})


install(
  TARGETS ${PROJECT_NAME} pose_node # add nodes
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)