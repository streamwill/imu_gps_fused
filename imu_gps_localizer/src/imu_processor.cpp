//
// created by streamwill on 2022.01.17
//
#include "imu_gps_localizer/imu_processor.h"
#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

ImuProcessor::ImuProcessor(const double acc_noise, const double gyro_noise,
                           const double acc_bias_noise, const double gyro_bias_noise,
                           const Eigen::Vector3d& gravity)
    : acc_noise_(acc_noise), gyro_noise_(gyro_noise), 
      acc_bias_noise_(acc_bias_noise), gyro_bias_noise_(gyro_bias_noise),
      gravity_(gravity) { }

void ImuProcessor::Predict(const ImuDataPtr cur_imu, State* state) {

    // Set last state.
    State last_state = *state;
    ImuDataPtr last_imu = std::make_shared<ImuData>();
    *last_imu = last_state.imu_data;

    // Time.
    const double delta_t = cur_imu->timestamp - last_imu->timestamp;
    const double delta_t2 = delta_t * delta_t;

    // Using P58. of "Quaternion kinematics for the error-state Kalman Filter".

    // Inertial solution Nominal-state
    state->acc_bias  = last_state.acc_bias;
    state->gyro_bias = last_state.gyro_bias;
    Eigen::Vector3d gyro_k0  = cur_imu->gyro - state->gyro_bias;
    Eigen::Vector3d gyro_k1 = last_imu->gyro - last_state.gyro_bias;

    Eigen::Vector3d delta_angle = 0.5 * (gyro_k1 + gyro_k0) * delta_t;
    Eigen::Matrix3d delta_rot   = Eigen::Matrix3d::Identity();
    // if (delta_angle.norm() < 1e-12){
    //     delta_rot = Eigen::Quaterniond(Eigen::Matrix3d::Identity() + GetSkewMatrix(delta_angle)).normalized().toRotationMatrix();
    // }else{
    //     delta_rot = Eigen::AngleAxisd(delta_angle.norm(), delta_angle.normalized()).toRotationMatrix();
    // }
    Sophus::SO3d bso3_dr = Sophus::SO3d::exp(delta_angle);
    delta_rot = bso3_dr.matrix();

    state->rot_imu2enu = last_state.rot_imu2enu * delta_rot;
    Eigen::Vector3d acc_k0 = cur_imu->acc - state->acc_bias;
    Eigen::Vector3d acc_k1 = last_imu->acc - last_state.acc_bias;
    Eigen::Vector3d acc_m = 0.5 * (state->rot_imu2enu * acc_k0 + last_state.rot_imu2enu * acc_k1);
    state->vel_imu2enu = last_state.vel_imu2enu + (acc_m + gravity_) * delta_t;
    state->poi_imu2enu = last_state.poi_imu2enu + 0.5 * (state->vel_imu2enu + last_state.vel_imu2enu) * delta_t;

    // Jacobians matrix of the error-state.   
    Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
    Fx.block<3, 3>(0, 3)  = Eigen::Matrix3d::Identity() * delta_t;
    Fx.block<3, 3>(3, 6)  = - state->rot_imu2enu * GetSkewMatrix(acc_k0) * delta_t;
    Fx.block<3, 3>(3, 9)  = - state->rot_imu2enu * delta_t;
    Fx.block<3, 3>(6 ,6)  = Eigen::Matrix3d::Identity() - GetSkewMatrix(gyro_k0);//delta_rot.transpose();
    Fx.block<3, 3>(6, 12) = - Eigen::Matrix3d::Identity() * delta_t;

    // Jacobians matrix of the perturbation vector
    Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

    // Covariances matrix of the perturbation impulses
    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = acc_noise_ * delta_t2 * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(3, 3) = gyro_noise_ * delta_t2 * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(6, 6) = acc_bias_noise_ * delta_t * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(9, 9) = gyro_bias_noise_ * delta_t * Eigen::Matrix3d::Identity();

    //*****[KF-1]*****Error-state
    state->error_state = Fx * last_state.error_state;

    //*****[KF-2]*****Covariances matrix of the error-state
    state->cov_es = Fx * last_state.cov_es * Fx.transpose() + Fi * Qi * Fi.transpose();

    // Time and imu.
    state->timestamp = cur_imu->timestamp;
    state->imu_data = *cur_imu;
}

}  // namespace ImuGpsLocalization