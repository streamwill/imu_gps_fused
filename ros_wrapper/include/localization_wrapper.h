#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "imu_gps_localizer/imu_gps_localizer.h"

class LocalizationWrapper {
public:
    LocalizationWrapper(ros::NodeHandle& nh);
    ~LocalizationWrapper();

    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr);

private:
    void LogState(const ImuGpsLocalization::State& state);
    void LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data);

    void ConvertStateToRosTopic(const ImuGpsLocalization::State& state);
    void PublisGPSPath(const ImuGpsLocalization::GpsPositionDataPtr gps_data);
    
    ros::Subscriber sub_imu_raw_;
    ros::Subscriber sub_gps_fix_;
    ros::Publisher pub_state_;
    ros::Publisher pub_gps_;

    std::ofstream file_state_;
    std::ofstream file_gps_;

    nav_msgs::Path path_state_;
    tf::TransformBroadcaster tfbc_state_;

    nav_msgs::Path path_gps_;
    tf::TransformBroadcaster tfbc_gps_;

    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_;
};