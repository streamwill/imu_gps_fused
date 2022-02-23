//
// created by streamwill on 2022.01.17
//
#include "imu_gps_localizer/gps_processor.h"
#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

GpsProcessor::GpsProcessor(const Eigen::Vector3d& poi_gps2imu) : poi_gps2imu_(poi_gps2imu) { }

bool GpsProcessor::UpdateStateByGpsPose(const GpsDataPtr gps_data_ptr, State* state) {

    // Compute observation:h_x and jacobian:H
    Eigen::Matrix<double, 6, 1 > h_x;
    Eigen::Matrix<double, 6, 15> H;
    ComputeObserveAndJacobian(gps_data_ptr->xyz, gps_data_ptr->qua, *state, &h_x, &H);
    
    //*****[KF-3]*****Compute kalman gain
    Eigen::Matrix<double, 6, 6> V;  V.setZero();
    V.block<3, 3>(0, 0) = gps_data_ptr->cov;
    V.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-6;  //covariance of attitude
    const Eigen::MatrixXd& P = state->cov_es;  //covariance matrix of error-state
    const Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + V).inverse();  //kalman gain

    //*****[KF-4]*****Update covarance matrix of error-state.
    const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
    state->cov_es = I_KH * P * I_KH.transpose() + K * V * K.transpose();

    //*****[KF-5]*****Update Error-state
    const Eigen::VectorXd delta_x = state->error_state + K * (h_x - H * state->error_state);

    // Update nominal state. X = X (+) delta_X.
    AddDeltaToState(delta_x, state);

    // Reset Error-state
    state->error_state.setZero();

    return true;
}

void GpsProcessor::ComputeObserveAndJacobian(const Eigen::Vector3d& poi_gps_enu, 
                                             const Eigen::Quaterniond& qua_gps_enu,
                                             const State& state,
                                             Eigen::Matrix<double, 6, 1 >* observe,
                                             Eigen::Matrix<double, 6, 15>* jacobian) {

    const Eigen::Vector3d& poi_imu2enu   = state.poi_imu2enu;
    const Eigen::Matrix3d& rot_imu2enu   = state.rot_imu2enu;

    // Compute residual.
    observe->setZero();
    observe->block<3, 1>(0, 0) = poi_gps_enu - (poi_imu2enu + rot_imu2enu * poi_gps2imu_);
    Eigen::Matrix3d delta_rot_mat = rot_imu2enu.transpose() * Eigen::Matrix3d(qua_gps_enu);
    Sophus::SO3d bso3_drot(delta_rot_mat);
    Eigen::Vector3d sso3_dang = bso3_drot.log();  //delta_ang must from SO3 to so3
    observe->block<3, 1>(3, 0) = sso3_dang;

    // Compute jacobian of residual by delta x. reference "quaternion kinematics for ESKF" page.42
    jacobian->setZero();
    jacobian->block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    jacobian->block<3, 3>(0, 6) = - rot_imu2enu * GetSkewMatrix(poi_gps2imu_);

    Eigen::Matrix3d t_skewMat = GetSkewMatrix(sso3_dang);
    double t_norm = sso3_dang.norm();
    Eigen::Matrix3d jr_1 = Eigen::Matrix3d::Zero();
    if(t_norm < 1.7e-7)
    {
        jr_1 = Eigen::Matrix3d::Identity() 
               + 0.5 * t_skewMat 
               + 0.25 * t_skewMat * t_skewMat;
    }else{
        jr_1 = Eigen::Matrix3d::Identity() 
               + 0.5 * t_skewMat
               + (1.0 / (t_norm * t_norm) - (1.0 + cos(t_norm)) / (2.0 * t_norm * sin(t_norm))) * t_skewMat * t_skewMat;
    }
    jacobian->block<3, 3>(3, 6) = jr_1;
}

void AddDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state) {
    state->poi_imu2enu += delta_x.block<3, 1>(0, 0);
    state->vel_imu2enu += delta_x.block<3, 1>(3, 0);

    Eigen::Vector3d delta_ang = delta_x.block<3, 1>(6, 0);
    Eigen::Matrix3d delta_rot   = Eigen::Matrix3d::Identity();
    // if (delta_ang.norm() < 1e-12) {
    //     delta_rot = Eigen::Quaterniond(Eigen::Matrix3d::Identity() + GetSkewMatrix(delta_ang)).normalized().toRotationMatrix();
    // } else {
    //     delta_rot = Eigen::AngleAxisd(delta_ang.norm(), delta_ang.normalized()).toRotationMatrix();
    // }
    Sophus::SO3d bso3_dr = Sophus::SO3d::exp(delta_ang);
    delta_rot = bso3_dr.matrix();

    state->rot_imu2enu *= delta_rot;
    state->acc_bias  += delta_x.block<3, 1>(9, 0);
    state->gyro_bias += delta_x.block<3, 1>(12, 0);
}

}  // namespace ImuGpsLocalization