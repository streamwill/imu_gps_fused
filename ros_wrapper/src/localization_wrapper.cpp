#include "localization_wrapper.h"
#include <glog/logging.h>

LocalizationWrapper::LocalizationWrapper(ros::NodeHandle& nh) {

    // subscriber
    sub_imu_raw_     = nh.subscribe("/imu_raw", 1000,  &LocalizationWrapper::ImuCallback, this);
    //sub_gps_fix_     = nh.subscribe("/fix", 10,  &LocalizationWrapper::GpsPositionCallback, this);
    sub_gps_nmea_    = nh.subscribe("/nmea_sentence", 100,  &LocalizationWrapper::NmeaCallback, this);
    sub_lidar_cloud_ = nh.subscribe("/points_raw", 10, &LocalizationWrapper::LidarCallback, this);

    // publisger
    pub_state_ = nh.advertise<nav_msgs::Path>("/path_state", 1);
    pub_gps_   = nh.advertise<nav_msgs::Path>("/path_gps", 1);
    pub_lidar_ = nh.advertise<nav_msgs::Path>("/path_lidar",1);
    pub_frame_ = nh.advertise<sensor_msgs::PointCloud2>("/frame_cloud",1);
    pub_local_map_   = nh.advertise<sensor_msgs::PointCloud2>("/local_map",1);
    pub_global_map_  = nh.advertise<sensor_msgs::PointCloud2>("/global_map",1);

    // Log configs.
    std::string log_folder = "/home/log";
    ros::param::get("log_folder", log_folder);
    log_folder.append("/log");
    if(access(log_folder.c_str(), 0) == -1)
    {
        auto unused = system((std::string("mkdir -p ") + log_folder).c_str());
    }
    FLAGS_log_dir = log_folder;
    file_state_.open(log_folder + "/state.csv");
    file_state_ << "timestamp" << "," 
                << "latitude" << "," << "longitude" << "," << "altitude" << ","
                << "position_x" << "," << "position_y" << "," << "position_z" << ","
                << "velocity_x" << "," << "velocity_y" << "," << "velocity_z" << ","
                << "roll" << "," << "piptch" << "," << "yaw" << "," 
                << "acc_bias_x" << "," << "acc_bias_y" << "," << "acc_bias_z" << ","
                << "gyro_bias_x" << "," << "gyro_bias_y" << "," << "gyro_bias_z" << "\n";
    file_gps_.open(log_folder +"/gps.csv");
    file_gps_ << "timestamp" << "," 
              << "latitude" << "," << "longitude" << "," << "altitude" << ","
              << "position_x" << "," << "position_y" << "," << "position_z" << ","
              << "roll" << "," << "pitch" << "," << "yaw" << "\n";
    file_lidar_.open(log_folder + "/lidar.csv");
    file_lidar_ << "timestamp" << "," 
                << "position_x" << "," << "position_y" << "," << "position_z" << ","
                << "roll" << "," << "pitch" << "," << "yaw" << "\n";

    // Load configs.
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
    nh.param("acc_noise",       acc_noise, 1e-2);
    nh.param("gyro_noise",      gyro_noise, 1e-4);
    nh.param("acc_bias_noise",  acc_bias_noise, 1e-6);
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);
    double x, y, z;
    nh.param("imu_p_gps_x", x, 0.);
    nh.param("imu_p_gps_y", y, 0.);
    nh.param("imu_p_gps_z", z, 0.);
    const Eigen::Vector3d imu_p_gps(x, y, z);
    nh.param("imu_p_lidar_x", x, 0.);
    nh.param("imu_p_lidar_y", y, 0.);
    nh.param("imu_p_lidar_z", z, 0.);
    const Eigen::Vector3d imu_p_lidar(x, y, z);

    //Cloud configs.
    map_global_ptr_.reset(new pcl::PointCloud<pointXYZI>());
    map_local_ptr_.reset(new pcl::PointCloud<pointXYZI>());
    cloud_RS_ptr_.reset(new pcl::PointCloud<pointXYZIRT>());
    cloud_in_ptr_.reset(new pcl::PointCloud<pointXYZI>());
    pose_last_ = Eigen::Matrix4d::Identity();
    pose_step_ = Eigen::Matrix4d::Identity();
    box_center_= Eigen::Vector3d::Zero();    
    // ndt_cpu
    ndt_ptr_.reset(new pcl::NormalDistributionsTransform<pointXYZI, pointXYZI>());
    ndt_ptr_->setResolution(5.0);
    ndt_ptr_->setStepSize(0.1);
    ndt_ptr_->setTransformationEpsilon(0.01);
    ndt_ptr_->setMaximumIterations(30);
    //ndt_gpu
    ndt_gpu_ptr_.reset(new gpu::GNormalDistributionsTransform());
    ndt_gpu_ptr_->setResolution(3.0);
    ndt_gpu_ptr_->setStepSize(0.1);
    ndt_gpu_ptr_->setMaximumIterations(30);
    ndt_gpu_ptr_->setTransformationEpsilon(0.01);

    // load map pcd file
    std::string map_path;
    ros::param::get("map_path", map_path);
    LoadCloudMap(map_path);

    has_init_gps_ = false;
    time_sta = std::chrono::system_clock::now();
    time_end = std::chrono::system_clock::now();    

    // Initialization imu gps localizer.
    imu_gps_localizer_ptr_ =  std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(
        acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise, imu_p_gps, imu_p_lidar);

}

LocalizationWrapper::~LocalizationWrapper() {
    file_state_.close();
    file_gps_.close();
    file_lidar_.close();
}

void LocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    time_sta = std::chrono::system_clock::now();
    ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
    imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x, 
                         imu_msg_ptr->linear_acceleration.y,
                         imu_msg_ptr->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
                          imu_msg_ptr->angular_velocity.y,
                          imu_msg_ptr->angular_velocity.z;
    
    ImuGpsLocalization::State fused_state;
    has_init_gps_ = imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state);
    if (!has_init_gps_) {
        return;
    }

    //pose_last_.block<3,3>(0,0) = fused_state.G_R_I;
    //pose_last_.block<3,1>(0,3) = fused_state.G_p_I;

    // Publish fused state.
    PublishSystemState(fused_state);

    // Log fused state.
    LogState(fused_state);
    time_end = std::chrono::system_clock::now();
    time_elapsed = time_end - time_sta;
    double time_duration = time_elapsed.count() * 1000.0;
    if(time_duration > 10.0){
        printf("[---Time:\033[1;31m%-8.3f\033[0mms]\n", time_duration);
    }
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

    has_init_gps_ = imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr);
    if(!has_init_gps_){
        return;
    }

    // publish gps state
    PublisGPSPath(gps_data_ptr);

    // log gps sate
    LogGps(gps_data_ptr);

}

void LocalizationWrapper::NmeaCallback(const nmea_msgs::SentenceConstPtr& nmea_msg_ptr){

    // static int num_now = 0;
    // std::chrono::duration<double> time_elapsed = time_end - time_sta;
    // printf("[---Frame:\033[1;33m%-4d\033[0m---Time:\033[1;33m%-8.3f\033[0mms]\r",
    //         num_now++, time_elapsed.count() * 1000.0);
    // fflush(stdout);  //刷新缓存区

    ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsPositionData>();
    gps_data_ptr->timestamp = nmea_msg_ptr->header.stamp.toSec(); 

    //split string of nmea sentence
    std::vector<std::string> sentence_vec;
    std::string token;
    std::stringstream strs(nmea_msg_ptr->sentence);
    while (std::getline(strs, token, ',')){
        sentence_vec.push_back(token);
    }
    
    //get lla
    try{
        if(sentence_vec.at(0) == "$GPAGM")
        {
            gps_data_ptr->lla << std::stod(sentence_vec.at(12)),
                                 std::stod(sentence_vec.at(13)),
                                 std::stod(sentence_vec.at(14)); 
            gps_data_ptr->cov = Eigen::Matrix3d::Zero();
            gps_data_ptr->rpy << std::stod(sentence_vec.at(5)),
                                 std::stod(sentence_vec.at(4)),
                                 std::stod(sentence_vec.at(3));
        }
    }
    catch(const std::exception& e){
        ROS_WARN_STREAM("GPS DATA TYPE is not '$GPAGM' !");
    }

    //process gps state
    has_init_gps_ = imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr);
    if(!has_init_gps_){
        return;
    }

    // publish gps state
    PublisGPSPath(gps_data_ptr);

    // log gps state
    LogGps(gps_data_ptr);
}

void LocalizationWrapper::LidarCallback(const sensor_msgs::PointCloud2::Ptr& lidar_msg_ptr){
    
    //init local map
    static bool has_local_map_(false);
    if(!has_local_map_){
        ResetLocalMap(box_center_(0), box_center_(1), box_center_(2));
        has_local_map_ = true;
    }    

    //view global map
    static bool has_global_map_(false);
    if(pub_global_map_.getNumSubscribers() > 0){
        if(!has_global_map_){
            sensor_msgs::PointCloud2Ptr msg_cloud_ptr(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*map_global_ptr_, *msg_cloud_ptr);
            msg_cloud_ptr->header.stamp = ros::Time::now();
            msg_cloud_ptr->header.frame_id = "map";
            pub_global_map_.publish(*msg_cloud_ptr);
            has_global_map_ = true;
        }
    }else{
        has_global_map_ = false;
    }

    //add lidar pose after init gps
    if(!has_init_gps_){
        return;
    }

    ImuGpsLocalization::LidarPoseDataPtr lidar_data_ptr = std::make_shared<ImuGpsLocalization::LidarPoseData>();
    lidar_data_ptr->timestamp = lidar_msg_ptr->header.stamp.toSec();

    Eigen::Matrix4d pose_matched;
    MatchFrame2Map(lidar_msg_ptr, pose_matched);

    lidar_data_ptr->pose = pose_matched;
    lidar_data_ptr->cov = Eigen::Matrix3d::Zero();

    //has_init_gps_ = imu_gps_localizer_ptr_->ProcessLidarPoseData(lidar_data_ptr);

    //publish lidar state
    PublishLidarPath(lidar_data_ptr);

    //log lidar state
    LogLidar(lidar_data_ptr);
}



void LocalizationWrapper::PublishSystemState(const ImuGpsLocalization::State& state) {
    path_state_.header.frame_id = "map";
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

void LocalizationWrapper::PublisGPSPath(const ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr){

    path_gps_.header.frame_id = "map";
    path_gps_.header.stamp = ros::Time(gps_data_ptr->timestamp);  
    const Eigen::Vector3d vec_xyz = gps_data_ptr->xyz;
    const Eigen::Vector3d vec_rpy = gps_data_ptr->rpy;

    geometry_msgs::PoseStamped pose_gps;
    pose_gps.header = path_gps_.header;
    pose_gps.pose.position.x = vec_xyz[0];
    pose_gps.pose.position.y = vec_xyz[1];
    pose_gps.pose.position.z = vec_xyz[2];
    pose_gps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(vec_rpy[0], vec_rpy[1], vec_rpy[2]);
    path_gps_.poses.push_back(pose_gps);

    pub_gps_.publish(path_gps_);
    
    // broadcaster tf of gps
    tf::Pose tf_pose_gps;
    tf::poseMsgToTF(pose_gps.pose, tf_pose_gps);

    tfbc_gps_.sendTransform(tf::StampedTransform(tf_pose_gps, pose_gps.header.stamp, pose_gps.header.frame_id, "gps"));

}

void LocalizationWrapper::PublishLidarPath(const ImuGpsLocalization::LidarPoseDataPtr lidar_data_ptr){

    path_lidar_.header.frame_id = "map";
    path_lidar_.header.stamp = ros::Time(lidar_data_ptr->timestamp);   
    const Eigen::Vector3d vec_xyz = lidar_data_ptr->pose.block<3,1>(0,3);
    const Eigen::Vector3d vec_rpy = lidar_data_ptr->pose.block<3,3>(0,0).eulerAngles(2,1,0);

    geometry_msgs::PoseStamped pose_lidar;
    pose_lidar.header = path_lidar_.header;
    pose_lidar.pose.position.x = vec_xyz[0];
    pose_lidar.pose.position.y = vec_xyz[1];
    pose_lidar.pose.position.z = vec_xyz[2];
    pose_lidar.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(vec_rpy[0], vec_rpy[1], vec_rpy[2]);
    path_lidar_.poses.push_back(pose_lidar);

    pub_lidar_.publish(path_lidar_);

    // broadcaster tf of lidar
    tf::Pose tf_pose_lidar;
    tf::poseMsgToTF(pose_lidar.pose, tf_pose_lidar);

    tfbc_lidar_.sendTransform(tf::StampedTransform(tf_pose_lidar, pose_lidar.header.stamp, pose_lidar.header.frame_id, "lidar"));
}

void LocalizationWrapper::LogState(const ImuGpsLocalization::State& state) {
    const Eigen::Quaterniond G_q_I(state.G_R_I);
    const Eigen::Vector3d geo_ea_imu = state.G_R_I.matrix().eulerAngles(2,1,0);
    file_state_ << std::fixed << std::setprecision(15)
                << state.timestamp << ","
                << state.lla[0] << "," << state.lla[1] << "," << state.lla[2] << ","
                << state.G_p_I[0] << "," << state.G_p_I[1] << "," << state.G_p_I[2] << ","
                << state.G_v_I[0] << "," << state.G_v_I[1] << "," << state.G_v_I[2] << ","
                << geo_ea_imu[0] << "," << geo_ea_imu[1] << "," << geo_ea_imu[2] << ","
                << state.acc_bias[0] << "," << state.acc_bias[1] << "," << state.acc_bias[2] << ","
                << state.gyro_bias[0] << "," << state.gyro_bias[1] << "," << state.gyro_bias[2] << "\n";
}

void LocalizationWrapper::LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr) {
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data_ptr->timestamp << ","
              << gps_data_ptr->lla[0] << "," << gps_data_ptr->lla[1] << "," << gps_data_ptr->lla[2] << ","
              << gps_data_ptr->xyz[0] << "," << gps_data_ptr->xyz[1] << "," << gps_data_ptr->xyz[2] << ","
              << gps_data_ptr->rpy[0] << "," << gps_data_ptr->rpy[1] << "," << gps_data_ptr->rpy[2] << "\n";
}

void LocalizationWrapper::LogLidar(const ImuGpsLocalization::LidarPoseDataPtr lidar_data_ptr){
    const Eigen::Vector3d vec_xyz = lidar_data_ptr->pose.block<3,1>(0,3);
    const Eigen::Vector3d vec_rpy = lidar_data_ptr->pose.block<3,3>(0,0).eulerAngles(2,1,0);
    file_lidar_ << std::fixed << std::setprecision(15)
                << lidar_data_ptr->timestamp << ","
                << vec_xyz[0] << "," << vec_xyz[1] << "," << vec_xyz[2] << ","
                << vec_rpy[0] << "," << vec_rpy[1] << "," << vec_rpy[2] << "\n";
}



void LocalizationWrapper::LoadCloudMap(const std::string& map_path){
    pcl::io::loadPCDFile(map_path, *map_global_ptr_);
    LOG(INFO) << "global map PATH: " << map_path.c_str();
    LOG(INFO) << "global map SIZE: " << map_global_ptr_->points.size();
    float leaf_size = 1.0;
    voxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter_.setInputCloud(map_global_ptr_);
    voxel_filter_.filter(*map_global_ptr_);
    LOG(INFO) << "global map voxel filter leaf SIZE: " << leaf_size;
}

void LocalizationWrapper::ResetLocalMap(float x, float y, float z){
    box_center_ << x, y, z;

    float box_size = 200.0;
    box_filter_.setMin(Eigen::Vector4f(x - box_size, y - box_size, z - box_size, 1.0e-6));
    box_filter_.setMax(Eigen::Vector4f(x + box_size, y + box_size, z + box_size, 1.0e-6));
    box_filter_.setInputCloud(map_global_ptr_);
    box_filter_.filter(*map_local_ptr_);

    double leaf_size = 2.0;
    voxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter_.setInputCloud(map_local_ptr_);
    voxel_filter_.filter(*map_local_ptr_);

    sensor_msgs::PointCloud2Ptr msg_cloud_ptr(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*map_local_ptr_, *msg_cloud_ptr);
    msg_cloud_ptr->header.stamp = ros::Time::now();
    msg_cloud_ptr->header.frame_id = "map";
    pub_local_map_.publish(*msg_cloud_ptr);

    LOG(INFO) << "new local map cropbox center: " 
              << box_center_(0) << ", " << box_center_(1) << ", " << box_center_(2);
}

void LocalizationWrapper::MatchFrame2Map(const sensor_msgs::PointCloud2::Ptr& lidar_msg_ptr, Eigen::Matrix4d& pose_frame){
    // parase rosmsgs
    cloud_RS_ptr_->clear();
    pcl::moveFromROSMsg(*lidar_msg_ptr, *cloud_RS_ptr_);
    cloud_in_ptr_->clear();
    for (size_t i = 0; i < cloud_RS_ptr_->size(); i++){
        auto &src = cloud_RS_ptr_->points[i];
        if (!pcl_isfinite(src.x) || !pcl_isfinite(src.y) || !pcl_isfinite(src.z)){
            continue;
        }
        pointXYZI point_this;
        point_this.x = src.x; point_this.y = src.y; point_this.z = src.z;
        point_this.intensity = src.intensity;
        cloud_in_ptr_->points.push_back(point_this);
    }
    cloud_in_ptr_->is_dense = true;
    
    //filter frame cloud
    float leaf_size = 2.0;
    voxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter_.setInputCloud(cloud_in_ptr_);
    voxel_filter_.filter(*cloud_in_ptr_);
    
    // update local map
    for (size_t i = 0; i < 3; i++)    {
        if(std::fabs(pose_last_(i, 3) - box_center_(i)) < 50.0){
            continue;
        }
        ResetLocalMap(pose_last_(0, 3), pose_last_(1, 3), pose_last_(2, 3));
        break;
    }
    
    //printf("\033[32m---map_size[%ld]---frame_size[%ld]\033[0m\n",map_local_ptr_->size(), cloud_in_ptr_->size());

    //frame match to map
    ndt_gpu_ptr_->setInputTarget(map_local_ptr_);
    ndt_gpu_ptr_->setInputSource(cloud_in_ptr_);
    cloudXYZI cloud_align;
    Eigen::Matrix4f pose_guess;
    pose_guess = pose_last_.cast<float>() * pose_step_.cast<float>();
    ndt_gpu_ptr_->align(pose_guess);
    pose_guess = ndt_gpu_ptr_->getFinalTransformation();
    pose_frame = pose_guess.cast<double>();

    pose_step_ = pose_last_.inverse() * pose_frame;
    pose_last_ = pose_frame;

    //publish aligned cloud
    cloudXYZI cloud_tfed;
    pcl::transformPointCloud(*cloud_in_ptr_, cloud_tfed, pose_frame);
    sensor_msgs::PointCloud2Ptr msg_cloud_ptr(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(cloud_tfed, *msg_cloud_ptr);
    msg_cloud_ptr->header.stamp = lidar_msg_ptr->header.stamp;
    msg_cloud_ptr->header.frame_id = "map";
    pub_frame_.publish(*msg_cloud_ptr);
}

