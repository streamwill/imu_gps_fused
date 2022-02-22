#include "imu_gps_localizer/initializer.h"
#include "imu_gps_localizer/utils.h"

#include <Eigen/Dense>
#include <glog/logging.h>

namespace ImuGpsLocalization {

Initializer::Initializer(const Eigen::Vector3d& pos_gps2imu_init) 
    : pos_gps2imu_(pos_gps2imu_init) { }

void Initializer::AddImuData(const ImuDataPtr imu_data_ptr) {
    imu_buffer_.push_back(imu_data_ptr);

    if (imu_buffer_.size() > imuBufferSize) {
        imu_buffer_.pop_front();
    }
}

bool Initializer::AddGpsData(const GpsDataPtr gps_data_ptr, State* state) {
    if (imu_buffer_.size() < imuBufferSize) {
        LOG(WARNING) << "[AddGpsData]: No enought imu data!";
        return false;
    }

    const ImuDataPtr last_imu_ptr = imu_buffer_.back();
    // TODO: synchronize all sensors.
    if (std::abs(gps_data_ptr->timestamp - last_imu_ptr->timestamp) > 0.5) {
        LOG(ERROR) << "[AddGpsData]: Gps and imu timestamps are not synchronized!";
        return false;
    }

    // Set timestamp
    state->timestamp = last_imu_ptr->timestamp;
    // Set imu data
    state->imu_data = *last_imu_ptr;
    // Set gps_lla data
    state->lla = gps_data_ptr->lla;

    // Set Error-state 
    state->error_state.setZero();

    // Set initial mean.
    state->poi_imu2enu.setZero();

    // We have no information to set initial velocity. 
    // So, just set it to zero and given big covariance.
    state->vel_imu2enu.setZero();

    // Currently set to Identity
    state->rot_imu2enu.setIdentity();

    // We can use the direction of gravity to set roll and pitch. 
    // But, we cannot set the yaw. 
    // So, we set yaw to zero and give it a big covariance.

    if (!ComputeRotIMU2ENUFromImuData(&state->rot_imu2enu)) {
        LOG(WARNING) << "[AddGpsData]: Failed to compute rot_imu2enu!";
        return false;
    }

    // Set bias to zero.
    state->acc_bias.setZero();
    state->gyro_bias.setZero();

    // Set state covariance matrix.
    state->cov_es.setZero();
    // position std: 10 m
    state->cov_es.block<3, 3>(0, 0) = 10. * 10. * Eigen::Matrix3d::Identity(); 
    // velocity std: 10 m/s
    state->cov_es.block<3, 3>(3, 3) = 10. * 10. * Eigen::Matrix3d::Identity(); 
    // rotation std: 10 degree, 10 degree, 100 degree
    state->cov_es(6, 6) = 10.  * kDegreeToRadian * 10.  * kDegreeToRadian;
    state->cov_es(7, 7) = 10.  * kDegreeToRadian * 10.  * kDegreeToRadian;
    state->cov_es(8, 8) = 100. * kDegreeToRadian * 100. * kDegreeToRadian;
    // Acc bias std:0.02 m/s^2
    state->cov_es.block<3, 3>(9, 9)   = 0.02 * 0.02 * Eigen::Matrix3d::Identity();
    // Gyro bias std:0.02 rad/s
    state->cov_es.block<3, 3>(12, 12) = 0.02 * 0.02 * Eigen::Matrix3d::Identity();

    return true;
}

bool Initializer::ComputeRotIMU2ENUFromImuData(Eigen::Matrix3d* rot_imu2enu) {
    // Compute mean and std of the imu buffer.
    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (const auto imu_data : imu_buffer_) {
        sum_acc += imu_data->acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();

    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (const auto imu_data : imu_buffer_) {
        sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    }
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();

    if (std_acc.maxCoeff() > accStdLimit) {
        LOG(WARNING) << "[ComputeRotIMU2ENUFromImuData]: Too big acc std: " << std_acc.transpose();
        return false;
    }

    // Compute rotation.
    // Please refer to 
    // https://github.com/rpng/open_vins/blob/master/ov_core/src/init/InertialInitializer.cpp
    
    // Three axises of the ENU frame in the IMU frame.
    
    // z-axis.
    const Eigen::Vector3d& z_axis = mean_acc.normalized(); 

    // x-axis.
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();

    // y-axis.
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();

    Eigen::Matrix3d rot_mat;
    rot_mat.block<3, 1>(0, 0) = x_axis;
    rot_mat.block<3, 1>(0, 1) = y_axis;
    rot_mat.block<3, 1>(0, 2) = z_axis;

    *rot_imu2enu = rot_mat.transpose();

    return true;
}

}  // namespace ImuGpsLocalization