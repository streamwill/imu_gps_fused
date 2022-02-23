cmake_minimum_required(VERSION 2.8.3)
project(imu_gps_localization)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf
  std_msgs nav_msgs geometry_msgs
  pcl_conversions
)
find_package(Eigen3  REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(CUDA REQUIRED)
find_package(Glog REQUIRED)

message("\n=========[CUDA INCLUDE] : ${CUDA_INCLUDE_DIRS}")
message("=========[CUDA LIBRARY] : ${CUDA_LIBRARIES}\n")

catkin_package()

add_subdirectory(imu_gps_localizer)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  imu_gps_localizer/include
  ros_wrapper/include
)
link_directories(
  ros_wrapper/lib
)


# build lib ros_wrapper_lib
add_library(ros_wrapper_lib
	ros_wrapper/src/localization_wrapper.cpp
)

target_link_libraries(ros_wrapper_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${GLOG_LIBRARIES}
  imu_gps_localizer_lib
)


# build exe imu_gps_localization_node
add_executable(${PROJECT_NAME}_node 
  ros_wrapper/src/localization_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${GLOG_LIBRARIES}
  ros_wrapper_lib
  ndt_gpu
)
