#pragma once

//CPP
#include <fstream>
#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <iomanip>
#include <glog/logging.h>

//ROS
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <ndt_gpu/NormalDistributionsTransform.h>

//LOCAL
#include "imu_gps_localizer/imu_gps_localizer.h"

//速腾雷达自定义点云类型(x,y,z,intensity,ring,time_stamp)
struct RobosensePointXYZIRT
{
    PCL_ADD_POINT4D;
    uint8_t intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RobosensePointXYZIRT,
                                  (float, x, x) (float, y, y) (float, z, z) (uint8_t, intensity, intensity)
                                  (uint16_t, ring, ring) (double, timestamp, timestamp));

typedef RobosensePointXYZIRT pointXYZIRT;
typedef pcl::PointXYZI pointXYZI;
typedef pcl::PointCloud<pointXYZI> cloudXYZI;

class LocalizationWrapper {
public:
    LocalizationWrapper(ros::NodeHandle& nh);
    ~LocalizationWrapper();

    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr);

    void NmeaCallback(const nmea_msgs::SentenceConstPtr& nmea_msg_ptr);

    void LidarCallback(const sensor_msgs::PointCloud2::Ptr& lidar_msg_ptr);

private:
    void LogState(const ImuGpsLocalization::State& state);
    void LogGps(const ImuGpsLocalization::GpsDataPtr gps_data_ptr);
    void LogLidar(const ImuGpsLocalization::LidarDataPtr lidar_data_ptr);

    void PublishSystemState(const ImuGpsLocalization::State& state);
    void PublisGPSPath(const ImuGpsLocalization::GpsDataPtr gps_data_ptr);
    void PublishLidarPath(const ImuGpsLocalization::LidarDataPtr lidar_data_ptr);

    void LoadCloudMap(const std::string& map_path);
    void ResetLocalMap(double x, double y, double z);
    void MatchFrame2Map(const sensor_msgs::PointCloud2::Ptr& lidar_msg_ptr, Eigen::Matrix4d& pose_frame);
    
    ros::Subscriber sub_imu_raw_;
    ros::Subscriber sub_gps_fix_;
    ros::Subscriber sub_gps_nmea_;
    ros::Subscriber sub_lidar_cloud_;

    ros::Publisher pub_state_;
    ros::Publisher pub_gps_;
    ros::Publisher pub_lidar_;
    ros::Publisher pub_local_map_;
    ros::Publisher pub_global_map_;
    ros::Publisher pub_frame_;

    std::ofstream file_state_;
    std::ofstream file_gps_;
    std::ofstream file_lidar_;

    nav_msgs::Path path_state_;
    nav_msgs::Path path_gps_;
    nav_msgs::Path path_lidar_;

    tf::TransformBroadcaster tfbc_state_;
    tf::TransformBroadcaster tfbc_gps_;
    tf::TransformBroadcaster tfbc_lidar_;

    pcl::PointCloud<pointXYZI>::Ptr map_global_ptr_;
    pcl::PointCloud<pointXYZI>::Ptr map_local_ptr_;
    pcl::PointCloud<pointXYZIRT>::Ptr cloud_RS_ptr_;    //速腾雷达
    pcl::PointCloud<pointXYZI>::Ptr cloud_in_ptr_;      //输入点云

    pcl::VoxelGrid<pointXYZI> voxel_filter_;
    pcl::CropBox<pointXYZI> box_filter_;
    double box_size_map_;
    double voxel_size_map_;
    double voxel_size_frame_;
    double ndt_resolution_;
    Eigen::Vector3d box_center_;
    Eigen::Matrix4d pose_last_;
    Eigen::Matrix4d pose_step_;
    
    pcl::NormalDistributionsTransform<pointXYZI, pointXYZI>::Ptr ndt_cpu_ptr_;  //ndt_cpu
    std::shared_ptr<gpu::GNormalDistributionsTransform>  ndt_gpu_ptr_;      //ndt_gpu
    pcl::GeneralizedIterativeClosestPoint<pointXYZI, pointXYZI>::Ptr gicp_ptr_;      //gicp

    bool has_init_gps_;
    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_;

    //time
    std::chrono::system_clock::time_point time_sta;
    std::chrono::system_clock::time_point time_end;
    std::chrono::duration<double> time_elapsed;
};