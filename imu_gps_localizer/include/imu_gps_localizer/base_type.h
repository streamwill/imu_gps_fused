#pragma once
#include <memory>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

namespace ImuGpsLocalization {

struct ImuData {
    double timestamp;      // In second.

    Eigen::Vector3d acc;   // Acceleration in m/s^2
    Eigen::Vector3d gyro;  // Angular velocity in radian/s.
};
using ImuDataPtr = std::shared_ptr<ImuData>;

struct GpsData {
    double timestamp;     // In second.
 
    Eigen::Vector3d lla;    // Latitude in degree, longitude in degree, and altitude in meter.
    Eigen::Matrix3d cov;    // Covariance in m^2.
    Eigen::Vector3d rpy;    //roll,pitch,yaw in degree, at NED frame

    Eigen::Vector3d xyz;    // x,y,z in m, at ENU frame.
    Eigen::Quaterniond qua; // x,y,z,w in quaternion ,at local frame
};
using GpsDataPtr = std::shared_ptr<GpsData>;

struct LidarData{
    double timestamp;       // In second.

    Eigen::Matrix4d pose;   // Transform matrix at map frame.
    Eigen::Matrix3d cov;    // Covariance of position TODO
};
using LidarDataPtr = std::shared_ptr<LidarData>;

struct State {
    // Time
    double timestamp;
    
    // Nominal-state
    Eigen::Vector3d poi_imu2enu;     // The original point of the IMU frame in the Global frame.
    Eigen::Vector3d vel_imu2enu;     // The velocity original point of the IMU frame in the Global frame.
    Eigen::Matrix3d rot_imu2enu;     // The rotation from the IMU frame to the Global frame.
    Eigen::Vector3d acc_bias;  // The bias of the acceleration sensor.
    Eigen::Vector3d gyro_bias; // The bias of the gyroscope sensor.

    // Error-state
    Eigen::Matrix<double, 15, 1> error_state;

    // Covariance of Error-state.
    Eigen::Matrix<double, 15, 15> cov_es;

    // IMU data.
    ImuData imu_data; 

    // GPS WGS84 position
    Eigen::Vector3d lla;

};

}  // ImuGpsLocalization