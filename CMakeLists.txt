cmake_minimum_required(VERSION 3.10.0)
project(imu_gps_localization)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp tf
  std_msgs nav_msgs 
)

find_package(Eigen3  REQUIRED)
find_package(Fmt REQUIRED)

set(GLOG_INCLUDE_DIRS /usr/local/include/glog)
set(GLOG_LIBRARIES /usr/local/lib/libglog.so)

# find_package(Glog REQUIRED)
message("=========[CMAKE_MODULE_PATH ] : ${CMAKE_MODULE_PATH}")
message("=========[GLOG_INCLUDE_DIRS ] : ${GLOG_INCLUDE_DIRS}")
message("=========[GLOG_LIBRARIES ] : ${GLOG_LIBRARIES}\n")

catkin_package()

add_subdirectory(imu_gps_localizer)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  imu_gps_localizer/include
  ros_wrapper/include
)

add_library(ros_wrapper_lib
	ros_wrapper/src/localization_wrapper.cpp
)
target_link_libraries(ros_wrapper_lib
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARIES}
  imu_gps_localizer_lib
)

## Localization node.
add_executable(${PROJECT_NAME}_node ros_wrapper/src/localization_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${GLOG_LIBRARIES}
  ros_wrapper_lib
)
