//
// created by streamwill on 2022.01.17
//
#pragma once
#include "imu_gps_localizer/base_type.h"

namespace ImuGpsLocalization{

    class LidarProcessor{
    public:
        LidarProcessor(const Eigen::Vector3d& poi_lidar2imu);
        ~LidarProcessor() = default;
        bool UpdateStateByLidarPose(const LidarDataPtr lidar_data_ptr, State* state);

    private:
        const Eigen::Vector3d poi_lidar2imu_;   //lidar position from imu frame
        
        void ComputeObserveAndJacobian(const Eigen::Vector3d& poi_lidar_enu, 
                                       const Eigen::Quaterniond& qua_lidar_enu,
                                       const State& state,
                                       Eigen::Matrix<double, 6, 1>* observe,
                                       Eigen::Matrix<double, 6, 15>* jacobian);
    };
    
    void AddLidarDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state);

} // namespace ImuGpsLocalization

