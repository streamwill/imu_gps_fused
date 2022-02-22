#pragma once
#include "imu_gps_localizer/base_type.h"
#include "imu_gps_localizer/gps_processor.h"
#include "imu_gps_localizer/imu_processor.h"
#include "imu_gps_localizer/lidar_processor.h"
#include "imu_gps_localizer/initializer.h"

#include <glog/logging.h>
#include <chrono>

namespace ImuGpsLocalization {

    class ImuGpsLocalizer {
    public:
        ImuGpsLocalizer(const double acc_noise, const double gyro_noise,
                        const double acc_bias_noise, const double gyro_bias_noise,
                        const Eigen::Vector3d& gravity_init,
                        const Eigen::Vector3d& poi_gps2imu,
                        const Eigen::Vector3d& poi_lidar2imu);

        bool ProcessImuData(const ImuDataPtr imu_data_ptr, State* fused_state);

        bool ProcessGpsData(const GpsDataPtr gps_data_ptr);

        bool ProcessLidarData(const LidarDataPtr lidar_data_ptr);

    private:
        std::unique_ptr<Initializer>  initializer_;
        std::unique_ptr<ImuProcessor> imu_processor_;
        std::unique_ptr<GpsProcessor> gps_processor_;
        std::unique_ptr<LidarProcessor> lidar_processor_;

        bool initialized_;
        Eigen::Vector3d init_lla_;  // The initial reference gps point.
        Eigen::Vector3d init_rpy_;  // The initial reference gps attitude.

        State state_;
    };

}  // namespace ImuGpsLocalization