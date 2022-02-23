//
// created by streamwill on 2022.01.17
//
#include "imu_gps_localizer/lidar_processor.h"
#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

    LidarProcessor::LidarProcessor(const Eigen::Vector3d& poi_lidar2imu) : poi_lidar2imu_(poi_lidar2imu) { }

    bool LidarProcessor::UpdateStateByLidarPose(const LidarDataPtr lidar_data_ptr, State* state){

        Eigen::Vector3d poi_lidar(lidar_data_ptr->pose.block<3,1>(0,3));
        //poi_lidar[2] = state->poi_imu2enu[2];   // z from ndt is bad
        Eigen::Quaterniond qua_lidar(lidar_data_ptr->pose.block<3,3>(0,0));
        qua_lidar = qua_lidar.normalized();
        
        // Compute observation:h_x and jacobian:H
        Eigen::Matrix<double, 6, 1 > h_x;
        Eigen::Matrix<double, 6, 15> H;
        ComputeObserveAndJacobian(poi_lidar, qua_lidar, *state, &h_x, &H);
        
        //*****[KF-3]*****Compute kalman gain
        Eigen::Matrix<double, 6, 6> V;  V.setZero();//covariance matrix of measurements
        V.block<3, 3>(0, 0) = lidar_data_ptr->cov;   
        V.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-6;   //covariance of attitude
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

        return true;
    }

    void LidarProcessor::ComputeObserveAndJacobian(const Eigen::Vector3d& poi_lidar_enu, 
                                                   const Eigen::Quaterniond& qua_lidar_enu,
                                                   const State& state,
                                                   Eigen::Matrix<double, 6, 1>* observe,
                                                   Eigen::Matrix<double, 6, 15>* jacobian){
        const Eigen::Vector3d& poi_imu2enu   = state.poi_imu2enu;
        const Eigen::Matrix3d& rot_imu2enu   = state.rot_imu2enu;
 
        // Compute residual.
        observe->setZero();
        observe->block<3, 1>(0, 0) = poi_lidar_enu - (poi_imu2enu - rot_imu2enu * poi_lidar2imu_); 
        Eigen::Matrix3d delta_rot_mat = rot_imu2enu.transpose() * Eigen::Matrix3d(qua_lidar_enu);
        Sophus::SO3d bso3_drot(delta_rot_mat);        
        Eigen::Vector3d sso3_dang = bso3_drot.log();  //delta_angle must from SO3 to so3        
        observe->block<3, 1>(3, 0) = sso3_dang;

        // Compute jacobian of residual by delta_X.
        jacobian->setZero();
        jacobian->block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        jacobian->block<3, 3>(0, 6) = - rot_imu2enu * GetSkewMatrix(poi_lidar2imu_);    
               
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

        // static int num_n = 1;
        // Eigen::Quaterniond qua_rot_imu(rot_imu2enu);
        // Eigen::Quaterniond qua_rot_lidar(qua_lidar_enu);
        // Eigen::Quaterniond qua_delta_rot(delta_rot_mat);
        // printf("\033[32m\n\n*****************[%d]\033[0m\n", num_n++);    // fflush(stdout);  //刷新缓存区
        // std::cout << "qua_rot_imu     : " << qua_rot_imu.coeffs().transpose() << std::endl;
        // std::cout << "qua_rot_lidar   : " << qua_rot_lidar.coeffs().transpose() << std::endl;
        // std::cout << "qua_delta_rot   : " << qua_delta_rot.coeffs().transpose() << std::endl;
        // std::cout << "sso3_dang_lidar : " << sso3_dang.transpose() << std::endl;
    }

    void AddLidarDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state) {
        state->poi_imu2enu += delta_x.block<3, 1>(0, 0);
        state->vel_imu2enu += delta_x.block<3, 1>(3, 0);

        Eigen::Vector3d delta_ang = delta_x.block<3, 1>(6, 0);
        Eigen::Matrix3d delta_rot = Eigen::Matrix3d::Identity();
        // if(delta_ang.norm() < 1e-12){
        //     delta_rot = Eigen::Quaterniond(Eigen::Matrix3d::Identity() + GetSkewMatrix(delta_ang)).normalized().toRotationMatrix();
        // }else
        // {
        //     delta_rot = Eigen::AngleAxisd(delta_ang.norm(), delta_ang.normalized()).toRotationMatrix();
        // }
        Sophus::SO3d bso3_dr = Sophus::SO3d::exp(delta_ang);
        delta_rot = bso3_dr.matrix();

        state->rot_imu2enu *= delta_rot;
        state->acc_bias  += delta_x.block<3,1>(9,0);
        state->gyro_bias += delta_x.block<3,1>(12,0);
    }
    
}