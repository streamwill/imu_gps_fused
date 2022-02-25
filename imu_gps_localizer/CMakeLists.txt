cmake_minimum_required(VERSION 2.8.3)
project(imu_gps_localizer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(Eigen3  REQUIRED)
find_package(GeographicLib REQUIRED)

message("=========[GeographicLib_INCLUDE_DIRS ] : ${GeographicLib_INCLUDE_DIRS}")
message("=========[GeographicLib_LIBRARIES ] : ${GeographicLib_LIBRARIES}\n")

# Add GeographicLib.
add_subdirectory(third_party/GeographicLib)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${GeographicLib_INCLUDE_DIRS}
)

add_library(imu_gps_localizer_lib
	src/imu_gps_localizer.cpp
  src/initializer.cpp
  src/imu_processor.cpp
  src/gps_processor.cpp
)

target_link_libraries(imu_gps_localizer_lib
	${EIGEN3_LIBS}
  ${GeographicLib_LIBRARIES}
)