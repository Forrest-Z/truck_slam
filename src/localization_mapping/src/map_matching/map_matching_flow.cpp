/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-04-13 19:22:34
 */
#include "localization_mapping/map_matching/map_matching_flow.hpp"
#include "glog/logging.h"


namespace truck_slam{
MapMatchingFlow::MapMatchingFlow(const std::string node_name, const rclcpp::NodeOptions &option) : Node(node_name, option){
    paramsInit();
    resetParametes();
    subAndPubInit();
    LOG(INFO) << "*************** Map Matching node Inited ***************\n";
}

void MapMatchingFlow::paramsInit(){
    // LOG(INFO) <<  "Parameters declare ... \n";
    this->declare_parameter("cloudInfoTopic" , "feature/cloud_info");
    this->declare_parameter("laserOdomTopic" , "map_matching/laser_odom");
    this->declare_parameter("gnssPoseTopic" ,  "/navsat/odom");
    this->declare_parameter("laserFrame" , "rslidar");
    this->declare_parameter("mapFrame" , "map");
    this->declare_parameter("useImuHeadingInitialization", false);
    this->declare_parameter("initMethod", "gnss");
}

void MapMatchingFlow::resetParametes(){
    // LOG(INFO) <<  "Parameters and variables reset ... \n";
    this->get_parameter("cloudInfoTopic" , cloud_info_topic_);
    this->get_parameter("laserOdomTopic" , laser_odom_topic_);
    this->get_parameter("gnssPoseTopic" , gnss_topic_);
    this->get_parameter("laserFrame" , laser_frame_);
    this->get_parameter("mapFrame" , map_frame_);
    this->get_parameter("useImuHeadingInitialization", use_imu_heading_initial_);
    this->get_parameter("initMethod", init_method_);

    map_matching_ptr_ = std::make_shared<MapMatching>();
    rviz_init_pose_arived_ = false;
    YAML::Node config_node = YAML::LoadFile(WORK_SPACE_PATH + "/config/map_localization.yaml");
    // create instance:
    scan_context_manager_ptr_ = std::make_shared<ScanContextManager>(config_node["map_matching_node"]["ros__parameters"]["scan_context"]);
}

void MapMatchingFlow::subAndPubInit(){
    LOG(INFO) <<  "Subscribers and Publishers init ... \n";
    callback_group1_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    callback_group2_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    callback_group3_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    
    auto sub_opt1 = rclcpp::SubscriptionOptions();
    auto sub_opt2 = rclcpp::SubscriptionOptions();
    auto sub_opt3 = rclcpp::SubscriptionOptions();
    
    sub_opt1.callback_group = callback_group1_;
    sub_opt2.callback_group = callback_group2_;
    sub_opt3.callback_group = callback_group3_;

    cloud_info_sub_ptr_ = this->create_subscription<msg_interface::msg::CloudInfo>(cloud_info_topic_ , 1000,
                                std::bind(&MapMatchingFlow::cloudInfoHandler, this, std::placeholders::_1), sub_opt1);
    gnss_sub_ptr_ = this->create_subscription<nav_msgs::msg::Odometry>(gnss_topic_, 1000,
                                std::bind(&MapMatchingFlow::gnssOdomHandler, this, std::placeholders::_1), sub_opt2);
    
    rviz_init_pose_sub_ptr_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1000,
                                std::bind(&MapMatchingFlow::rvizInitHandler, this, std::placeholders::_1), sub_opt3);
    
    laser_odom_pub_ptr_ = this->create_publisher<nav_msgs::msg::Odometry>(laser_odom_topic_, 100);
    corner_global_map_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_matching/corner_global_map", 100);
    surf_global_map_pub_ptr_   = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_matching/surf_global_map", 100);
    corner_local_map_pub_ptr_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_matching/corner_local_map", 100);
    surf_local_map_pub_ptr_    = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_matching/surf_local_map", 100);
    tf_odom_to_laser_ptr_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

bool MapMatchingFlow::initPose(){
    if(map_matching_ptr_->isInited()) return true;
    float init_pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    if(init_method_ == "gnss"){
        if(use_imu_heading_initial_){
        init_pose[0] = current_cloud_info_.imu_roll_init;
        init_pose[1] = current_cloud_info_.imu_pitch_init;
        init_pose[2] = current_cloud_info_.imu_yaw_init;
        }else{
            double cloud_time = rclcpp::Time(current_cloud_info_.header.stamp).seconds();
            nav_msgs::msg::Odometry gnss_data;
            while(!gnss_deque_.empty()){
                gnss_data = gnss_deque_.front();
                double gnss_time = rclcpp::Time(gnss_data.header.stamp).seconds();
                // 找到于当前帧较近的gnss数据作为初始化
                LOG(INFO) << "gnss_time : " << gnss_time << " , cloud_time: " << cloud_time << std::endl;
                if(std::fabs(gnss_time - cloud_time) < 0.2)
                    break;
                gnss_deque_.pop_back();
            }

            if(gnss_deque_.empty()){
                LOG(INFO) << "Can not find valid gnss data, return ... \n";
                return false;
            }
            double roll = 0.0, pitch = 0.0, yaw = 0.0;
            tf2::Quaternion q(gnss_data.pose.pose.orientation.x, gnss_data.pose.pose.orientation.y,
                            gnss_data.pose.pose.orientation.z, gnss_data.pose.pose.orientation.w);
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            init_pose[0] = roll; init_pose[1] = pitch; init_pose[2] = yaw;
            init_pose[3] = gnss_data.pose.pose.position.x; 
            init_pose[4] = gnss_data.pose.pose.position.y; 
            init_pose[5] = gnss_data.pose.pose.position.z; 
        }
    }else if(init_method_ == "scan_context"){
        // if(!setScanContextPose(current_cloud_info_, init_pose)){
        //     LOG(INFO) << "Scan context detect failed ! ... " << std::endl;
        //     return false;
        // }
    }else if(init_method_ == "rviz"){
        if(!rviz_init_pose_arived_){
            LOG(INFO) << "Waiting for RVIZ initial pose ... \n";        
            return false;
        }
        for(int i = 0 ; i < 6; i++)
            init_pose[i] = rviz_init_pose_[i];
    }else {
        LOG(INFO) << "Error, Init method must be gnss or scan_context ... \n";
        return false;
    }

    map_matching_ptr_->setInitPose(init_pose);  
    return true;
}

bool MapMatchingFlow::setScanContextPose(const msg_interface::msg::CloudInfo &cloud_info, float* pose){
    // get init pose proposal using scan context match:
    Eigen::Matrix4f init_pose = pcl::getTransformation(pose[3], pose[4], pose[5], pose[0], pose[1], pose[2]).matrix();  
    pcl::PointCloud<PointType>::Ptr scan(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(cloud_info.cloud_surface, *scan);
    if(!scan_context_manager_ptr_->detectLoopClosure(*scan, init_pose)) return false;
    // transform
    truck_slam:: transToArray(init_pose, pose);
    return true;
}


void MapMatchingFlow::rvizInitHandler(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr rviz_init_pose){
    if(rviz_init_pose_arived_) return;
    LOG(INFO) << "rviz init pose coming ... \n";
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Quaternion q(rviz_init_pose->pose.pose.orientation.x, rviz_init_pose->pose.pose.orientation.y,
                        rviz_init_pose->pose.pose.orientation.z, rviz_init_pose->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    rviz_init_pose_[0] = roll;
    rviz_init_pose_[1] = pitch;
    rviz_init_pose_[2] = yaw;
    rviz_init_pose_[3] = rviz_init_pose->pose.pose.position.x;
    rviz_init_pose_[4] = rviz_init_pose->pose.pose.position.y;
    rviz_init_pose_[5] = rviz_init_pose->pose.pose.position.z;

    for(int i = 0; i < 6; i++)
        LOG(INFO) << "rviz_init_pose_ " << rviz_init_pose_[i] << std::endl;
    rviz_init_pose_arived_ = true;
}

void MapMatchingFlow::cloudInfoHandler(const msg_interface::msg::CloudInfo::SharedPtr cloud_info){
    std::lock_guard<std::mutex> lock(cloud_info_lock_);
    // 得到当前cloud info数据
    current_cloud_info_ = *cloud_info;
    map_matching_ptr_->setCloudInfo(current_cloud_info_);
    
    // 系统初始化
    if(!initPose()) return;

    // 前端里程计匹配求解pose 
    auto t1 = std::chrono::system_clock::now();
    updateLaserOdometry();
    auto t2 = std::chrono::system_clock::now();
    LOG(INFO) << "updateLaserOdometry cost time : " << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() << " ms." << std::endl;

    // 发布前端激光里程计    
    publishData();
}

void MapMatchingFlow::gnssOdomHandler(const nav_msgs::msg::Odometry::SharedPtr gnss_data){
    std::lock_guard<std::mutex> lock(gnss_lock_);
    if(!map_matching_ptr_->isInited())
        gnss_deque_.push_back(*gnss_data);
}

void MapMatchingFlow::updateLaserOdometry(){
    // 设置位姿初始值
    map_matching_ptr_->updateInitialGuess();
    
    // 构建local map，用于匹配
    map_matching_ptr_->buildLocalMap();

    // 将当前帧角点和平面点降采样
    map_matching_ptr_->downsampleCurrentScan(current_cloud_info_);
    
    // scan to map优化得到激光里程计 
    is_update_ = map_matching_ptr_->mapMatch();
}

void MapMatchingFlow::publishData(){
    // publish global map    
    sensor_msgs::msg::PointCloud2 corner_global_map;
    sensor_msgs::msg::PointCloud2 surf_global_map;
    map_matching_ptr_->getGlobalMap(corner_global_map, surf_global_map);
    corner_global_map.header.frame_id = map_frame_;
    surf_global_map.header.frame_id = map_frame_;
    corner_global_map_pub_ptr_->publish(corner_global_map);
    surf_global_map_pub_ptr_->publish(surf_global_map); 

    // publish local map    
    sensor_msgs::msg::PointCloud2 corner_from_map;
    sensor_msgs::msg::PointCloud2 surf_from_map;
    map_matching_ptr_->getFeatureFromMap(corner_from_map, surf_from_map);
    corner_from_map.header.frame_id = map_frame_;
    surf_from_map.header.frame_id = map_frame_;
    corner_local_map_pub_ptr_->publish(corner_from_map);
    surf_local_map_pub_ptr_->publish(surf_from_map); 

    if(is_update_){
        // pub laser odom
        nav_msgs::msg::Odometry laser_odom;
        laser_odom.header.stamp = current_cloud_info_.header.stamp;
        laser_odom.header.frame_id = map_frame_;       // odom map
        laser_odom.child_frame_id = laser_frame_;    // laser odom
        laser_odom.pose.pose.position.x =  map_matching_ptr_->getPosition().x();
        laser_odom.pose.pose.position.y =  map_matching_ptr_->getPosition().y();
        laser_odom.pose.pose.position.z =  map_matching_ptr_->getPosition().z();
        tf2::Quaternion q; q.setRPY(map_matching_ptr_->getEulerAngles().x(), map_matching_ptr_->getEulerAngles().y(), map_matching_ptr_->getEulerAngles().z());
        laser_odom.pose.pose.orientation.w = q.w();
        laser_odom.pose.pose.orientation.x = q.x();
        laser_odom.pose.pose.orientation.y = q.y();
        laser_odom.pose.pose.orientation.z = q.z();
        laser_odom_pub_ptr_->publish(laser_odom);

        // odom to lidar tf 
        geometry_msgs::msg::TransformStamped odom_to_laser;
        odom_to_laser.header = laser_odom.header;
        odom_to_laser.child_frame_id = laser_odom.child_frame_id;
        odom_to_laser.transform.translation.x = laser_odom.pose.pose.position.x;
        odom_to_laser.transform.translation.y = laser_odom.pose.pose.position.y;
        odom_to_laser.transform.translation.z = laser_odom.pose.pose.position.z;
        odom_to_laser.transform.rotation.w = laser_odom.pose.pose.orientation.w;
        odom_to_laser.transform.rotation.x = laser_odom.pose.pose.orientation.x;
        odom_to_laser.transform.rotation.y = laser_odom.pose.pose.orientation.y;
        odom_to_laser.transform.rotation.z = laser_odom.pose.pose.orientation.z;
        tf_odom_to_laser_ptr_->sendTransform(odom_to_laser);
    }
}
}