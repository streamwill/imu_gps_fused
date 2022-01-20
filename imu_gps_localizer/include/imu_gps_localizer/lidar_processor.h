//
// created by streamwill on 2022.01.17
//
#pragma once

#include <Eigen/Dense>

#include "imu_gps_localizer/base_type.h"

namespace ImuGpsLocalization{

    class LidarProcessor
    {
    public:
        LidarProcessor(const Eigen::Vector3d& imu_p_lidar);
        ~LidarProcessor() = default;
        bool UpdateStateByLidarPose(const LidarPoseDataPtr lidar_data_ptr, State* state);


    private:
        const Eigen::Vector3d imu_p_lidar_;   //lidar position from imu frame
        
        void ComputerJacobianAndResidual(const Eigen::Vector3d& Geo_p_Lidar,
                                         const State& state,
                                         Eigen::Matrix<double, 3, 15>* jacobian,
                                         Eigen::Vector3d* residual);
    };
    

    void AddLidarDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state);

} // namespace ImuGpsLocalization

