cmake_minimum_required(VERSION 3.10.0)
project(imu_gps_localizer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(Eigen3  REQUIRED)
find_package(GeographicLib REQUIRED)

find_package(Fmt REQUIRED)
find_package(Sophus REQUIRED)

message("=========[GeographicLib_INCLUDE_DIRS ] : ${GeographicLib_INCLUDE_DIRS}")
message("=========[GeographicLib_LIBRARIES ] : ${GeographicLib_LIBRARIES}\n")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  )

# build lib imu_gps_localizer_lib
add_library(imu_gps_localizer_lib
  src/imu_processor.cpp
  src/gps_processor.cpp
  src/lidar_processor.cpp
  src/initializer.cpp
  src/imu_gps_localizer.cpp
  )

target_link_libraries(imu_gps_localizer_lib
  ${GeographicLib_LIBRARIES}
  ${GLOG_LIBRARIES}
  )