//
// created by streamwill on 2022.01.17
//
#include "imu_gps_localizer/lidar_processor.h"
#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

    LidarProcessor::LidarProcessor(const Eigen::Vector3d& poi_lidar2imu) : poi_lidar2imu_(poi_lidar2imu) { }

    bool LidarProcessor::UpdateStateByLidarPose(const LidarDataPtr lidar_data_ptr, State* state){
        Eigen::Matrix4d pose_g2lidar = lidar_data_ptr->pose;
        pose_g2lidar(2, 3) = state->poi_imu2enu[2];       // z from ndt is bad

        // Compute observation:h_x and jacobian:H
        Eigen::Matrix<double, 6, 1 > h_x;
        Eigen::Matrix<double, 6, 15> H;
        ComputeObserveAndJacobian(pose_g2lidar, *state, &h_x, &H);
        
        //*****[KF-3]*****Compute kalman gain
        Eigen::Matrix<double, 6, 6> V;  V.setZero();//covariance matrix of measurements
        V.block<3, 3>(0, 0) = lidar_data_ptr->cov;   
        V.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.1;
        const Eigen::MatrixXd& P = state->cov_es;          //covariance matrix of error-state
        const Eigen::MatrixXd  K = P * H.transpose() * (H * P * H.transpose() + V).inverse();   //kalman gain

        //*****[KF-4]*****Update covarance matrix of error-state.
        const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
        state->cov_es = I_KH * P * I_KH.transpose() + K * V * K.transpose();

        //*****[KF-5]*****Update Error-state
        const Eigen::VectorXd  delta_X = state->error_state + K * (h_x - H * state->error_state);
        
        // Update nominal state. X = X (+) delta_X.
        AddLidarDeltaToState(delta_X, state);

        // Reset Error-state
        state->error_state.setZero();
    }

    void LidarProcessor::ComputeObserveAndJacobian(const Eigen::Matrix4d& pose,
                                                   const State& state,
                                                   Eigen::Matrix<double, 6, 1>* observe,
                                                   Eigen::Matrix<double, 6, 15>* jacobian){
        const Eigen::Vector3d& pos_g2imu = state.poi_imu2enu;
        const Eigen::Matrix3d& rot_g2imu = state.rot_imu2enu;
        const Eigen::Vector3d& pos_g2lidar = pose.block<3, 1>(0, 3);
        const Eigen::Matrix3d& rot_g2lidar = pose.block<3, 3>(0, 0);

        // Compute residual.
        observe->setZero();
        observe->block<3, 1>(0, 0) = pos_g2lidar - (pos_g2imu - rot_g2imu * poi_lidar2imu_); 
        Eigen::Matrix3d delta_rot_mat = rot_g2imu.transpose() * rot_g2lidar;
        Eigen::Vector3d delta_eula_ang = delta_rot_mat.eulerAngles(2, 1, 0).reverse();
        for (size_t i = 0; i < 3; i++)
        {
            double angle_i = delta_eula_ang[i];
            if(angle_i >  M_PI / 2)
            {
                delta_eula_ang[i] = -M_PI + angle_i;
            }

            if(angle_i < -M_PI / 2)
            {
                delta_eula_ang[i] = M_PI + angle_i;
            }
        }
        observe->block<3, 1>(0, 3) = delta_eula_ang;
        // static int num_n = 1;
        // std::cout << "\n\n*****************" << num_n++ << std::endl;
        // std::cout << "delta_rot_mat: \n" << delta_rot_mat << std::endl;
        // std::cout << "delta_eula_ang: \n" << delta_eula_ang << std::endl;

        // Compute jacobian of residual by delta_X.
        jacobian->setZero();
        jacobian->block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        jacobian->block<3, 3>(0, 6) = - rot_g2imu * GetSkewMatrix(poi_lidar2imu_);    
        jacobian->block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();    
    }

    void AddLidarDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state) {
        state->poi_imu2enu += delta_x.block<3, 1>(0, 0);
        state->vel_imu2enu += delta_x.block<3, 1>(3, 0);

        Eigen::Vector3d d_angle = delta_x.block<3, 1>(6, 0);
        Eigen::Matrix3d d_rotmat = Eigen::Matrix3d::Identity();
        if(d_angle.norm() < 1e-12){
            d_rotmat = Eigen::Quaterniond(Eigen::Matrix3d::Identity() + GetSkewMatrix(d_angle)).normalized().toRotationMatrix();
        }else
        {
            d_rotmat = Eigen::AngleAxisd(d_angle.norm(), d_angle.normalized()).toRotationMatrix();
        }
        state->rot_imu2enu *= d_rotmat;

        state->acc_bias  += delta_x.block<3,1>(9,0);
        state->gyro_bias += delta_x.block<3,1>(12,0);
    }
    
}