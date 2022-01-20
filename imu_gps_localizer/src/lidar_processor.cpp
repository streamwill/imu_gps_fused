//
// created by streamwill on 2022.01.17
//
#include "imu_gps_localizer/lidar_processor.h"
#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

    LidarProcessor::LidarProcessor(const Eigen::Vector3d& imu_p_lidar) : imu_p_lidar_(imu_p_lidar) { }

    bool LidarProcessor::UpdateStateByLidarPose(const LidarPoseDataPtr lidar_data_ptr, State* state){
        //get position of lidar in map frame
        Eigen::Vector3d Map_p_Lidar; 
        Map_p_Lidar = lidar_data_ptr->pose.block<3,1>(0,3);

        // Compute residual and iacobian of residual
        Eigen::Matrix<double, 3, 15> H;     //jacobian matrix of measurements
        Eigen::Vector3d residual;           //residual between observation and last prediction
        ComputerJacobianAndResidual(Map_p_Lidar, *state, &H, &residual);
        const Eigen::Matrix3d& V = lidar_data_ptr->cov;     //covariance matrix of measurements

        // ESKF.
        const Eigen::MatrixXd& P = state->cov;  //covariance matrix of error-state
        const Eigen::MatrixXd  K = P * H.transpose() * (H * P * H.transpose() + V).inverse();   //kalman gain
        const Eigen::VectorXd  delta_X = K * residual;   //delta

        // Update nominal state. X = X (+) delta_X.
        AddLidarDeltaToState(delta_X, state);

        // Update covariance matrix of error-state.
        const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
        state->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
    }

    void LidarProcessor::ComputerJacobianAndResidual(const Eigen::Vector3d& Geo_p_Lidar,
                                     const State& state,
                                     Eigen::Matrix<double, 3, 15>* jacobian,
                                     Eigen::Vector3d* residual){
        const Eigen::Vector3d& geo_p_imu = state.G_p_I;
        const Eigen::Matrix3d& geo_r_imu = state.G_R_I;

        // Compute residual.
        *residual = Geo_p_Lidar - (geo_p_imu + geo_r_imu * imu_p_lidar_);

        // Compute jacobian of residual by delta_X.
        jacobian->setZero();
        jacobian->block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        jacobian->block<3, 3>(0, 6) = - geo_r_imu * GetSkewMatrix(imu_p_lidar_);        
    }

    void AddLidarDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state) {
        state->G_p_I += delta_x.block<3,1>(0,0);
        state->G_v_I += delta_x.block<3,1>(3,0);
        if(delta_x.block<3,1>(6,0).norm() > 1e-12){
            state->G_R_I *= Eigen::AngleAxisd(delta_x.block<3,1>(6,0).norm(), delta_x.block<3,1>(6,0).normalized()).toRotationMatrix();
        }
        state->acc_bias  += delta_x.block<3,1>(9,0);
        state->gyro_bias += delta_x.block<3,1>(12,0);
    }
    
}