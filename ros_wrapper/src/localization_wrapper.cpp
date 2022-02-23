#include "localization_wrapper.h"

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
    file_state_ << "timestamp"   << "," 
                << "position_x"  << "," << "position_y"  << "," << "position_z"  << ","
                << "velocity_x"  << "," << "velocity_y"  << "," << "velocity_z"  << ","
                << "attitude_qx" << "," << "attitude_qy" << "," << "attitude_qz" << "," << "attitude_qw" << ","
                << "acc_bias_x"  << "," << "acc_bias_y"  << "," << "acc_bias_z"  << ","
                << "gyro_bias_x" << "," << "gyro_bias_y" << "," << "gyro_bias_z" << ","
                << "latitude"    << "," << "longitude"   << "," << "altitude"    << "\n";
    file_state_.flush();
    file_gps_.open(log_folder +"/gps.csv");
    file_gps_ << "timestamp"   << "," 
              << "latitude"    << "," << "longitude"   << "," << "altitude"    << ","
              << "roll"        << "," << "pitch"       << "," << "yaw"         <<  ","
              << "position_x"  << "," << "position_y"  << "," << "position_z"  << ","
              << "attitude_qx" << "," << "attitude_qy" << "," << "attitude_qz" << "," << "attitude_qw" << "\n";
    file_gps_.flush();
    file_lidar_.open(log_folder + "/lidar.csv");
    file_lidar_ << "timestamp"   << "," 
                << "position_x"  << "," << "position_y"  << "," << "position_z"  << ","
                << "attitude_qx" << "," << "attitude_qy" << "," << "attitude_qz" << "," << "attitude_qw" << "\n";
    file_lidar_.flush();

    // Load configs.
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
    nh.param("acc_noise",       acc_noise, 1e-2);
    nh.param("gyro_noise",      gyro_noise, 1e-4);
    nh.param("acc_bias_noise",  acc_bias_noise, 1e-6);
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);
    double x, y, z;
    nh.param("poi_gps2imu_x", x, 0.);
    nh.param("poi_gps2imu_y", y, 0.);
    nh.param("poi_gps2imu_z", z, 0.);
    const Eigen::Vector3d poi_gps2imu(x, y, z);
    nh.param("poi_lidar2imu_x", x, 0.);
    nh.param("poi_lidar2imu_y", y, 0.);
    nh.param("poi_lidar2imu_z", z, 0.);
    const Eigen::Vector3d poi_lidar2imu(x, y, z);
    const Eigen::Vector3d vec_gravity0(0.0, 0.0, -9.794778);  //init gravity at enu frame
    nh.param("box_size_map", box_size_map_, 0.);
    nh.param("voxel_size_map", voxel_size_map_, 0.);
    nh.param("voxel_size_frame", voxel_size_frame_, 0.);
    nh.param("ndt_resolution", ndt_resolution_, 0.);

    //Cloud configs.
    map_global_ptr_.reset(new pcl::PointCloud<pointXYZI>());
    map_local_ptr_.reset(new pcl::PointCloud<pointXYZI>());
    cloud_RS_ptr_.reset(new pcl::PointCloud<pointXYZIRT>());
    cloud_in_ptr_.reset(new pcl::PointCloud<pointXYZI>());
    pose_last_ = Eigen::Matrix4d::Identity();
    pose_step_ = Eigen::Matrix4d::Identity();
    box_center_= Eigen::Vector3d::Zero();    
    // ndt_cpu
    ndt_cpu_ptr_.reset(new pcl::NormalDistributionsTransform<pointXYZI, pointXYZI>());
    ndt_cpu_ptr_->setResolution(ndt_resolution_);
    ndt_cpu_ptr_->setStepSize(0.1);
    ndt_cpu_ptr_->setTransformationEpsilon(0.01);
    ndt_cpu_ptr_->setMaximumIterations(30);
    // ndt_gpu
    ndt_gpu_ptr_.reset(new gpu::GNormalDistributionsTransform());
    ndt_gpu_ptr_->setResolution(ndt_resolution_);
    ndt_gpu_ptr_->setStepSize(0.1);
    ndt_gpu_ptr_->setMaximumIterations(30);
    ndt_gpu_ptr_->setTransformationEpsilon(0.01);
    // gicp
    gicp_ptr_.reset(new pcl::GeneralizedIterativeClosestPoint<pointXYZI, pointXYZI>());
    gicp_ptr_->setMaxCorrespondenceDistance(50);
    gicp_ptr_->setMaximumIterations(30);
    gicp_ptr_->setEuclideanFitnessEpsilon(1e-6);
    gicp_ptr_->setRANSACIterations(0);
    gicp_ptr_->setTransformationEpsilon(1e-6);

    // load map pcd file
    std::string map_path;
    ros::param::get("map_path", map_path);
    LoadCloudMap(map_path);

    has_init_gps_ = false;
    time_sta = std::chrono::system_clock::now();
    time_end = std::chrono::system_clock::now();    

    // Initialization imu gps localizer.
    imu_gps_localizer_ptr_ =  std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(
        acc_noise, gyro_noise, 
        acc_bias_noise, gyro_bias_noise,
        vec_gravity0, poi_gps2imu, poi_lidar2imu);

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

    // update pose to lidar
    pose_last_.block<3,3>(0,0) = fused_state.rot_imu2enu;
    pose_last_.block<3,1>(0,3) = fused_state.poi_imu2enu;

    // Publish fused state.
    PublishSystemState(fused_state);

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

    ImuGpsLocalization::GpsDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsData>();
    gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
    gps_data_ptr->lla << gps_msg_ptr->latitude,
                         gps_msg_ptr->longitude,
                         gps_msg_ptr->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());

    has_init_gps_ = imu_gps_localizer_ptr_->ProcessGpsData(gps_data_ptr);
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

    ImuGpsLocalization::GpsDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsData>();
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
            gps_data_ptr->cov = Eigen::Matrix3d::Identity() * 1e-4;
            gps_data_ptr->rpy << std::stod(sentence_vec.at(5)),
                                 std::stod(sentence_vec.at(4)),
                                 std::stod(sentence_vec.at(3));
        }
    }
    catch(const std::exception& e){
        ROS_WARN_STREAM("GPS DATA TYPE is not '$GPAGM' !");
    }

    //process gps state
    has_init_gps_ = imu_gps_localizer_ptr_->ProcessGpsData(gps_data_ptr);

    // return when has not initialized
    if(!has_init_gps_){
        return;
    }

    // publish gps state
    PublisGPSPath(gps_data_ptr);

    // log gps state
    LogGps(gps_data_ptr);
}

void LocalizationWrapper::LidarCallback(const sensor_msgs::PointCloud2::Ptr& lidar_msg_ptr){
    
    //add lidar pose after init gps
    if(!has_init_gps_){
        return;
    }

    //init local map
    static bool has_local_map_(false);
    if(!has_local_map_){
        box_center_ = pose_last_.block<3,1>(0,3);
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

    ImuGpsLocalization::LidarDataPtr lidar_data_ptr = std::make_shared<ImuGpsLocalization::LidarData>();
    lidar_data_ptr->timestamp = lidar_msg_ptr->header.stamp.toSec();

    Eigen::Matrix4d pose_matched;
    MatchFrame2Map(lidar_msg_ptr, pose_matched);

    lidar_data_ptr->pose = pose_matched;
    lidar_data_ptr->cov  = Eigen::Matrix3d::Identity() * 1e-4;

    imu_gps_localizer_ptr_->ProcessLidarData(lidar_data_ptr);

    //publish lidar state
    PublishLidarPath(lidar_data_ptr);

    //log lidar state
    LogLidar(lidar_data_ptr);
}



void LocalizationWrapper::PublishSystemState(const ImuGpsLocalization::State& state) {
    path_state_.header.frame_id = "map";
    path_state_.header.stamp = ros::Time(state.timestamp);  

    geometry_msgs::PoseStamped pose_state;
    pose_state.header = path_state_.header;

    const Eigen::Vector3d poi_state(state.poi_imu2enu);
    pose_state.pose.position.x = poi_state[0];
    pose_state.pose.position.y = poi_state[1];
    pose_state.pose.position.z = poi_state[2];

    const Eigen::Quaterniond qua_state(state.rot_imu2enu);
    pose_state.pose.orientation.x = qua_state.x();
    pose_state.pose.orientation.y = qua_state.y();
    pose_state.pose.orientation.z = qua_state.z();
    pose_state.pose.orientation.w = qua_state.w();

    while(path_state_.poses.size() > 5000)
    {
        path_state_.poses.erase(path_state_.poses.begin());
    }
    path_state_.poses.push_back(pose_state);

    pub_state_.publish(path_state_);

    // broadcaster tf of fused state  
    tf::Pose tf_pose_state;
    tf::poseMsgToTF(pose_state.pose, tf_pose_state);
    tfbc_state_.sendTransform(tf::StampedTransform(tf_pose_state, pose_state.header.stamp, pose_state.header.frame_id, "state"));
}

void LocalizationWrapper::PublisGPSPath(const ImuGpsLocalization::GpsDataPtr gps_data_ptr){

    path_gps_.header.frame_id = "map";
    path_gps_.header.stamp = ros::Time(gps_data_ptr->timestamp);  

    geometry_msgs::PoseStamped pose_gps;
    pose_gps.header = path_gps_.header;

    const Eigen::Vector3d poi_gps(gps_data_ptr->xyz);
    pose_gps.pose.position.x = poi_gps[0];
    pose_gps.pose.position.y = poi_gps[1];
    pose_gps.pose.position.z = poi_gps[2];

    const Eigen::Quaterniond qua_gps(gps_data_ptr->qua);
    pose_gps.pose.orientation.x = qua_gps.x();
    pose_gps.pose.orientation.y = qua_gps.y();
    pose_gps.pose.orientation.z = qua_gps.z();
    pose_gps.pose.orientation.w = qua_gps.w();

    path_gps_.poses.push_back(pose_gps);

    pub_gps_.publish(path_gps_);
    
    // broadcaster tf of gps
    tf::Pose tf_pose_gps;
    tf::poseMsgToTF(pose_gps.pose, tf_pose_gps);
    tfbc_gps_.sendTransform(tf::StampedTransform(tf_pose_gps, pose_gps.header.stamp, pose_gps.header.frame_id, "gps"));

}

void LocalizationWrapper::PublishLidarPath(const ImuGpsLocalization::LidarDataPtr lidar_data_ptr){

    path_lidar_.header.frame_id = "map";
    path_lidar_.header.stamp = ros::Time(lidar_data_ptr->timestamp);   

    geometry_msgs::PoseStamped pose_lidar;
    pose_lidar.header = path_lidar_.header;

    const Eigen::Vector3d poi_lidar(lidar_data_ptr->pose.block<3,1>(0,3));
    pose_lidar.pose.position.x = poi_lidar[0];
    pose_lidar.pose.position.y = poi_lidar[1];
    pose_lidar.pose.position.z = poi_lidar[2];

    const Eigen::Quaterniond qua_lidar(lidar_data_ptr->pose.block<3,3>(0,0));
    pose_lidar.pose.orientation.x = qua_lidar.x();
    pose_lidar.pose.orientation.y = qua_lidar.y();
    pose_lidar.pose.orientation.z = qua_lidar.z();
    pose_lidar.pose.orientation.w = qua_lidar.w();

    path_lidar_.poses.push_back(pose_lidar);

    pub_lidar_.publish(path_lidar_);

    // broadcaster tf of lidar
    tf::Pose tf_pose_lidar;
    tf::poseMsgToTF(pose_lidar.pose, tf_pose_lidar);
    tfbc_lidar_.sendTransform(tf::StampedTransform(tf_pose_lidar, pose_lidar.header.stamp, pose_lidar.header.frame_id, "lidar"));
}

void LocalizationWrapper::LogState(const ImuGpsLocalization::State& state) {
    const Eigen::Quaterniond qua_state(state.rot_imu2enu);
    file_state_ << std::fixed << std::setprecision(15)
                << state.timestamp << ","
                << state.poi_imu2enu[0] << "," << state.poi_imu2enu[1] << "," << state.poi_imu2enu[2] << ","
                << state.vel_imu2enu[0] << "," << state.vel_imu2enu[1] << "," << state.vel_imu2enu[2] << ","
                << qua_state.x()        << "," << qua_state.y()        << "," << qua_state.z()        << "," << qua_state.w() << ","
                << state.acc_bias[0]    << "," << state.acc_bias[1]    << "," << state.acc_bias[2]    << ","
                << state.gyro_bias[0]   << "," << state.gyro_bias[1]   << "," << state.gyro_bias[2]   << ","
                << state.lla[0]         << "," << state.lla[1]         << "," << state.lla[2]         << "\n";
    file_state_.flush();
}

void LocalizationWrapper::LogGps(const ImuGpsLocalization::GpsDataPtr gps_data_ptr) {
    const Eigen::Quaterniond qua_gps = gps_data_ptr->qua;
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data_ptr->timestamp << ","
              << gps_data_ptr->lla[0] << "," << gps_data_ptr->lla[1] << "," << gps_data_ptr->lla[2] << ","
              << gps_data_ptr->rpy[0] << "," << gps_data_ptr->rpy[1] << "," << gps_data_ptr->rpy[2] << ","
              << gps_data_ptr->xyz[0] << "," << gps_data_ptr->xyz[1] << "," << gps_data_ptr->xyz[2] << ","
              << qua_gps.x()          << "," << qua_gps.y()          << "," << qua_gps.z()          << "," << qua_gps.w() << "\n";
    file_gps_.flush();
}

void LocalizationWrapper::LogLidar(const ImuGpsLocalization::LidarDataPtr lidar_data_ptr){
    const Eigen::Vector3d poi_lidar(lidar_data_ptr->pose.block<3,1>(0,3));
    const Eigen::Quaterniond qua_lidar(lidar_data_ptr->pose.block<3,3>(0,0));
    file_lidar_ << std::fixed << std::setprecision(15)
                << lidar_data_ptr->timestamp << ","
                << poi_lidar[0]  << "," << poi_lidar[1]  << "," << poi_lidar[2]  << ","
                << qua_lidar.x() << "," << qua_lidar.y() << "," << qua_lidar.z() << "," << qua_lidar.w() << "\n";
    file_lidar_.flush();
}



void LocalizationWrapper::LoadCloudMap(const std::string& map_path){
    pcl::io::loadPCDFile(map_path, *map_global_ptr_);
    LOG(INFO) << "[global map PATH]: " << map_path.c_str();
    LOG(INFO) << "[global map SIZE]: " << map_global_ptr_->points.size();
    double leaf_size = 1.0;
    voxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter_.setInputCloud(map_global_ptr_);
    voxel_filter_.filter(*map_global_ptr_);
    LOG(INFO) << "[global map voxel filter leaf SIZE]: " << leaf_size;
}

void LocalizationWrapper::ResetLocalMap(double x, double y, double z){
    box_center_ << x, y, z;

    double box_size = box_size_map_;
    box_filter_.setMin(Eigen::Vector4f(x - box_size, y - box_size, z - box_size, 1.0e-6));
    box_filter_.setMax(Eigen::Vector4f(x + box_size, y + box_size, z + box_size, 1.0e-6));
    box_filter_.setInputCloud(map_global_ptr_);
    box_filter_.filter(*map_local_ptr_);

    double leaf_size = voxel_size_map_;
    voxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter_.setInputCloud(map_local_ptr_);
    voxel_filter_.filter(*map_local_ptr_);

    sensor_msgs::PointCloud2Ptr msg_cloud_ptr(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*map_local_ptr_, *msg_cloud_ptr);
    msg_cloud_ptr->header.stamp = ros::Time::now();
    msg_cloud_ptr->header.frame_id = "map";
    pub_local_map_.publish(*msg_cloud_ptr);

    //LOG(INFO) << "[new local map cropbox center]: " << box_center_(0) << ", " << box_center_(1) << ", " << box_center_(2);
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
    double leaf_size = voxel_size_frame_;
    voxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter_.setInputCloud(cloud_in_ptr_);
    voxel_filter_.filter(*cloud_in_ptr_);
    
    // update local map
    for (size_t i = 0; i < 3; i++)    {
        if(std::fabs(pose_last_(i, 3) - box_center_(i)) < 30.0){
            continue;
        }
        ResetLocalMap(pose_last_(0, 3), pose_last_(1, 3), pose_last_(2, 3));
        break;
    }
    
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

