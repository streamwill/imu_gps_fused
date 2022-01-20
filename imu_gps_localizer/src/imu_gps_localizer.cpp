#include "imu_gps_localizer/imu_gps_localizer.h"

#include <glog/logging.h>
#include <chrono>

#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

ImuGpsLocalizer::ImuGpsLocalizer(const double acc_noise, const double gyro_noise,
                                 const double acc_bias_noise, const double gyro_bias_noise,
                                 const Eigen::Vector3d& I_p_Gps,
                                 const Eigen::Vector3d& imu_p_lidar) 
    : initialized_(false){
    initializer_ = std::make_unique<Initializer>(I_p_Gps);
    imu_processor_ = std::make_unique<ImuProcessor>(acc_noise, gyro_noise, 
                                                    acc_bias_noise, gyro_bias_noise,
                                                    Eigen::Vector3d(0., 0., -9.81007));
    gps_processor_ = std::make_unique<GpsProcessor>(I_p_Gps);
    lidar_processor_ = std::make_unique<LidarProcessor>(imu_p_lidar);
}

bool ImuGpsLocalizer::ProcessImuData(const ImuDataPtr imu_data_ptr, State* fused_state) {
    if (!initialized_) {
        initializer_->AddImuData(imu_data_ptr);
        return false;
    }
    
    // Predict.
    imu_processor_->Predict(state_.imu_data_ptr, imu_data_ptr, &state_);

    // Convert fused ENU state to lla.
    ConvertENUToLLA(init_lla_, state_.G_p_I, &(state_.lla));
    *fused_state = state_;
    return true;
}

bool ImuGpsLocalizer::ProcessGpsPositionData(const GpsPositionDataPtr gps_data_ptr) {
    if (!initialized_) {
        if (!initializer_->AddGpsPositionData(gps_data_ptr, &state_)) {
            return false;
        }

        // Initialize the initial gps point used to convert lla to ENU.
        init_lla_ = gps_data_ptr->lla;
        
        //Initialize the initial state pose by initial gps point.
        init_rpy_ = gps_data_ptr->rpy;

        initialized_ = true;
        LOG(INFO) << "[ProcessGpsPositionData]: System initialized!";
        return true;
    }

    // Convert wgs84 lla to ENU frame.
    Eigen::Vector3d xyz_gps;
    ConvertLLAToENU(init_lla_, gps_data_ptr->lla, &xyz_gps);
    gps_data_ptr->xyz = xyz_gps;

    // convert enu to map
    ConvertENU2BODY(init_rpy_, gps_data_ptr->xyz, gps_data_ptr->rpy);

    // Update pose by gps.
    gps_processor_->UpdateStateByGpsPosition(gps_data_ptr, &state_);

    return true;
}

bool ImuGpsLocalizer::ProcessLidarPoseData(const LidarPoseDataPtr lidar_data_ptr){
    if(!initialized_){
        return false;
    }

    // update pose by lidar.
    lidar_processor_->UpdateStateByLidarPose(lidar_data_ptr, &state_);
    
    return true;
}

}  // namespace ImuGpsLocalization