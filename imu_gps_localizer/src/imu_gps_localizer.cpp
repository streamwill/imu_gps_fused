#include "imu_gps_localizer/imu_gps_localizer.h"
#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

    ImuGpsLocalizer::ImuGpsLocalizer(const double acc_noise, const double gyro_noise,
                                     const double acc_bias_noise, const double gyro_bias_noise,
                                     const Eigen::Vector3d& vec_gravity0,
                                     const Eigen::Vector3d& poi_gps2imu, 
                                     const Eigen::Vector3d& poi_lidar2imu){
        initialized_   = false;
        initializer_   = std::make_unique<Initializer>(poi_gps2imu);

        imu_processor_ = std::make_unique<ImuProcessor>(acc_noise, gyro_noise, 
                                                        acc_bias_noise, gyro_bias_noise,
                                                        vec_gravity0);
        gps_processor_   = std::make_unique<GpsProcessor>(poi_gps2imu);
        lidar_processor_ = std::make_unique<LidarProcessor>(poi_lidar2imu);
    }

    bool ImuGpsLocalizer::ProcessImuData(const ImuDataPtr imu_data_ptr, State* fused_state) {
        if (!initialized_) {
            initializer_->AddImuData(imu_data_ptr);
            return false;
        }
        
        // predict.
        imu_processor_->Predict(imu_data_ptr, &state_);

        // convert xyz to lla (enu frame to wgs84 frame).
        ConvertENUToLLA(init_lla_, state_.poi_imu2enu, state_.lla);

        // update.
        *fused_state = state_;
        return true;
    }

    bool ImuGpsLocalizer::ProcessGpsData(const GpsDataPtr gps_data_ptr) {
        if (!initialized_) {
            if (!initializer_->AddGpsData(gps_data_ptr, &state_)) {
                return false;
            }

            // Initialize the initial gps point used to convert lla to ENU.
            init_lla_ = gps_data_ptr->lla;
            
            //Initialize the initial state pose by initial gps point.
            init_rpy_ = gps_data_ptr->rpy;

            initialized_ = true;
            LOG(INFO) << "[ProcessGpsData]: System initialized!";
            //return true;
        }

        // convert lla to xyz (wgs84 frame to enu frame).
        ConvertLLAToENU(init_lla_, gps_data_ptr->lla, gps_data_ptr->xyz);

        // convert poi&rot form enu to local 
        ConvertENU2BODY(init_rpy_, gps_data_ptr->rpy, gps_data_ptr->xyz, gps_data_ptr->qua);

        // Update pose by gps.
        gps_processor_->UpdateStateByGpsPose(gps_data_ptr, &state_);

        return true;
    }

    bool ImuGpsLocalizer::ProcessLidarData(const LidarDataPtr lidar_data_ptr){
        if(!initialized_){
            return false;
        }

        // update pose by lidar.
        lidar_processor_->UpdateStateByLidarPose(lidar_data_ptr, &state_);
        
        return true;
    }

}  // namespace ImuGpsLocalization