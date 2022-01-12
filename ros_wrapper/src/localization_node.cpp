#include <memory>

#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "localization_wrapper.h"

int main (int argc, char** argv) {
    // Set glog.
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr  = true;
    FLAGS_colorlogtostderr = true;

    // Initialize ros.
    ros::init(argc, argv, "imu_gps_localization");
    ros::NodeHandle nh;
    
    // Initialize localizer.
    LocalizationWrapper localizer(nh);

    ros::spin();

    google::ShutdownGoogleLogging();
    return 1;
}