#include "localization_wrapper.h"

#include <iomanip>

#include <glog/logging.h>

#include "imu_gps_localizer/base_type.h"

LocalizationWrapper::LocalizationWrapper(ros::NodeHandle& nh) {
    // Load configs.
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
    nh.param("acc_noise",       acc_noise, 1e-2);
    nh.param("gyro_noise",      gyro_noise, 1e-4);
    nh.param("acc_bias_noise",  acc_bias_noise, 1e-6);
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);

    double x, y, z;
    nh.param("I_p_Gps_x", x, 0.);
    nh.param("I_p_Gps_y", y, 0.);
    nh.param("I_p_Gps_z", z, 0.);
    const Eigen::Vector3d I_p_Gps(x, y, z);

    std::string log_folder = "/home/log";
    ros::param::get("log_folder", log_folder);
    log_folder.append("/log");

    if(access(log_folder.c_str(), 0) == -1)
    {
        auto unused = system((std::string("mkdir -p ") + log_folder).c_str());
    }

    // Log.
    FLAGS_log_dir = log_folder;
    file_state_.open(log_folder + "/state.csv");
    file_state_ << "timestamp" << "," 
                << "latitude" << "," << "longitude" << "," << "altitude" << ","
                << "position_x" << "," << "position_y" << "," << "position_z" << ","
                << "velocity_x" << "," << "velocity_y" << "," << "velocity_z" << ","
                << "attitude_x" << "," << "attitude_y" << "," << "attitude_z" << "," << "attitude_w" << "," 
                << "acc_bias_x" << "," << "acc_bias_y" << "," << "acc_bias_z" << ","
                << "gyro_bias_x" << "," << "gyro_bias_y" << "," << "gyro_bias_z" << "\n";
    file_gps_.open(log_folder +"/gps.csv");
    file_gps_ << "timestamp" << "," 
              << "latitude" << "," << "longitude" << "," << "altitude" << ","
              << "position_x" << "," << "position_y" << "," << "position_z" << "\n";

    // Initialization imu gps localizer.
    imu_gps_localizer_ptr_ = 
        std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(acc_noise, gyro_noise,
                                                              acc_bias_noise, gyro_bias_noise,
                                                              I_p_Gps);

    // Subscribe topics.
    sub_imu_raw_ = nh.subscribe("/imu/data", 10,  &LocalizationWrapper::ImuCallback, this);
    sub_gps_fix_ = nh.subscribe("/fix", 10,  &LocalizationWrapper::GpsPositionCallback, this);

    pub_state_ = nh.advertise<nav_msgs::Path>("path_state", 1);
    pub_gps_   = nh.advertise<nav_msgs::Path>("path_gps", 1);
}

LocalizationWrapper::~LocalizationWrapper() {
    file_state_.close();
    file_gps_.close();
}

void LocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
    imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x, 
                         imu_msg_ptr->linear_acceleration.y,
                         imu_msg_ptr->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
                          imu_msg_ptr->angular_velocity.y,
                          imu_msg_ptr->angular_velocity.z;
    
    ImuGpsLocalization::State fused_state;
    bool has_init = imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state);
    if (!has_init) {
        return;
    }

    // Publish fused state.
    ConvertStateToRosTopic(fused_state);

    // Log fused state.
    LogState(fused_state);
}

void LocalizationWrapper::GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr) {
    //static int gps_c = 1;
    //printf("\033[33m---gps_frame[%6d]\033[0m\n",gps_c++);

    // Check the gps_status.
    if (gps_msg_ptr->status.status != 2) {
        LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
        return;
    }

    ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsPositionData>();
    gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
    gps_data_ptr->lla << gps_msg_ptr->latitude,
                         gps_msg_ptr->longitude,
                         gps_msg_ptr->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());

    bool has_init = imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr);
    if(!has_init){
        return;
    }

    // publish gps state
    PublisGPSPath(gps_data_ptr);

    // log gps sate
    LogGps(gps_data_ptr);
}



void LocalizationWrapper::ConvertStateToRosTopic(const ImuGpsLocalization::State& state) {
    path_state_.header.frame_id = "world";
    path_state_.header.stamp = ros::Time(state.timestamp);  

    geometry_msgs::PoseStamped pose;
    pose.header = path_state_.header;

    pose.pose.position.x = state.G_p_I[0];
    pose.pose.position.y = state.G_p_I[1];
    pose.pose.position.z = state.G_p_I[2];

    const Eigen::Quaterniond G_q_I(state.G_R_I);
    pose.pose.orientation.x = G_q_I.x();
    pose.pose.orientation.y = G_q_I.y();
    pose.pose.orientation.z = G_q_I.z();
    pose.pose.orientation.w = G_q_I.w();

    path_state_.poses.push_back(pose);
    pub_state_.publish(path_state_);

    // broadcaster tf of fused state
    tf::Pose tf_pose_state;
    tf::poseMsgToTF(pose.pose, tf_pose_state);
    tfbc_state_.sendTransform(tf::StampedTransform(tf_pose_state, pose.header.stamp, pose.header.frame_id, "state"));
}

void LocalizationWrapper::LogState(const ImuGpsLocalization::State& state) {
    const Eigen::Quaterniond G_q_I(state.G_R_I);
    file_state_ << std::fixed << std::setprecision(15)
                << state.timestamp << ","
                << state.lla[0] << "," << state.lla[1] << "," << state.lla[2] << ","
                << state.G_p_I[0] << "," << state.G_p_I[1] << "," << state.G_p_I[2] << ","
                << state.G_v_I[0] << "," << state.G_v_I[1] << "," << state.G_v_I[2] << ","
                << G_q_I.x() << "," << G_q_I.y() << "," << G_q_I.z() << "," << G_q_I.w() << ","
                << state.acc_bias[0] << "," << state.acc_bias[1] << "," << state.acc_bias[2] << ","
                << state.gyro_bias[0] << "," << state.gyro_bias[1] << "," << state.gyro_bias[2] << "\n";
}



void LocalizationWrapper::PublisGPSPath(const ImuGpsLocalization::GpsPositionDataPtr gps_data){

    path_gps_.header.frame_id = "world";
    path_gps_.header.stamp = ros::Time(gps_data->timestamp);  

    geometry_msgs::PoseStamped pose;
    pose.header = path_gps_.header;

    pose.pose.position.x = gps_data->xyz[0];
    pose.pose.position.y = gps_data->xyz[1];
    pose.pose.position.z = gps_data->xyz[2];

    pose.pose.orientation.x = 0.;
    pose.pose.orientation.y = 0.;
    pose.pose.orientation.z = 0.;
    pose.pose.orientation.w = 1.;

    path_gps_.poses.push_back(pose);
    pub_gps_.publish(path_gps_);
    
    // broadcaster tf of fused state
    tf::Pose tf_pose_gps;
    tf::poseMsgToTF(pose.pose, tf_pose_gps);
    tfbc_gps_.sendTransform(tf::StampedTransform(tf_pose_gps, pose.header.stamp, pose.header.frame_id, "gps"));

}

void LocalizationWrapper::LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data) {
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data->timestamp << ","
              << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << ","
              << gps_data->xyz[0] << "," << gps_data->xyz[1] << "," << gps_data->xyz[2] << "\n";
}

