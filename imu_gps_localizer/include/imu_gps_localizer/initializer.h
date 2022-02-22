#pragma once
#include "imu_gps_localizer/base_type.h"
#include <deque>

namespace ImuGpsLocalization {

    constexpr int imuBufferSize = 100;
    constexpr float accStdLimit = 3.0;

    class Initializer {
    public:
        Initializer(const Eigen::Vector3d& pos_gps2imu);
        
        void AddImuData(const ImuDataPtr imu_data_ptr);

        bool AddGpsData(const GpsDataPtr gps_data_ptr, State* state);

    private:
        bool ComputeRotIMU2ENUFromImuData(Eigen::Matrix3d* rot_imu2enu);

        Eigen::Vector3d pos_gps2imu_;
        std::deque<ImuDataPtr> imu_buffer_;
    };

}  // namespace ImuGpsLocalization