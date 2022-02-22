//
// created by streamwill on 2022.01.17
//
#pragma once
#include "imu_gps_localizer/base_type.h"

namespace ImuGpsLocalization {

    class GpsProcessor {
    public:
        GpsProcessor(const Eigen::Vector3d& poi_gps2imu);
        ~GpsProcessor() = default;
        bool UpdateStateByGpsPose(const GpsDataPtr gps_data_ptr, State* state);

    private: 
        const Eigen::Vector3d poi_gps2imu_; 

        void ComputeObserveAndJacobian(const Eigen::Vector3d& poi_gps_enu, 
                                       const Eigen::Quaterniond& qua_gps_enu,
                                       const State& state,
                                       Eigen::Matrix<double, 6, 1 >* observe,
                                       Eigen::Matrix<double, 6, 15 >* jacobian);
    };

    void AddDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state);

}  // namespace ImuGpsLocalization