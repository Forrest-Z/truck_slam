/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-13 09:32:38
 */

#include "localization_mapping/front_end/front_end_flow.hpp"
#include "glog/logging.h"

namespace truck_slam{

FrontEndFlow::FrontEndFlow(const std::string node_name, const rclcpp::NodeOptions &option)
                : Node(node_name, option){
    paramsInit();
    resetParametes();
    subAndPubInit();
    LOG(INFO) << "*************** Front End node Inited ***************\n";
}


void FrontEndFlow::paramsInit(){
    // LOG(INFO) <<  "Parameters declare ... \n";
    YAML::Node config_node = YAML::LoadFile(WORK_SPACE_PATH + "/config/config.yaml");
    // topic name
    cloud_info_topic_ = config_node["topic"]["cloudInfoTopic"].as<std::string>();
    laser_odom_topic_ = config_node["topic"]["laserOdomTopic"].as<std::string>();
    base_odom_topic_ = config_node["topic"]["baseOdomTopic"].as<std::string>();
    opt_key_frame_topic_ = config_node["topic"]["keyFrameTopic"].as<std::string>();
    gnss_topic_ = config_node["topic"]["gnssPoseTopic"].as<std::string>();
    laser_frame_pose_topic_ = config_node["topic"]["laserFramePoseTopic"].as<std::string>();
    // frame
    odom_frame_ = config_node["frame"]["odometryFrame"].as<std::string>();
    map_frame_ = config_node["frame"]["mapFrame"].as<std::string>();
    base_frame_ = config_node["frame"]["baselinkFrame"].as<std::string>();
    // gnss settings
    use_imu_heading_initial_ = config_node["gnss"]["useImuHeadingInitialization"].as<bool>();
    // init method
    init_method_ = config_node["initMethod"].as<std::string>();
}

void FrontEndFlow::resetParametes(){
    // LOG(INFO) <<  "Parameters and variables reset ... \n";
    front_end_ptr_ = std::make_shared<FrontEnd>();
}

void FrontEndFlow::subAndPubInit(){
    // LOG(INFO) <<  "Subscribers and Publishers init ... \n";
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
                                std::bind(&FrontEndFlow::cloudInfoHandler, this, std::placeholders::_1), sub_opt1);
    opt_key_frames_sub_ptr_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(opt_key_frame_topic_, 1000,
                                std::bind(&FrontEndFlow::optKeyFramesHandler, this, std::placeholders::_1), sub_opt2);
    gnss_sub_ptr_ = this->create_subscription<nav_msgs::msg::Odometry>(gnss_topic_, 1000,
                                std::bind(&FrontEndFlow::gnssOdomHandler, this, std::placeholders::_1), sub_opt3);
    laser_pose_pub_ptr_ = this->create_publisher<msg_interface::msg::KeyFramePose>(laser_frame_pose_topic_, 100);
    laser_odom_pub_ptr_ = this->create_publisher<nav_msgs::msg::Odometry>(laser_odom_topic_, 100);
    base_odom_pub_ptr_ = this->create_publisher<nav_msgs::msg::Odometry>(base_odom_topic_, 100);

    // service
    service_ = this->create_service<msg_interface::srv::SaveMap>("save_map", std::bind(&FrontEndFlow::saveMap, this, std::placeholders::_1, std::placeholders::_2));

    // debug
    current_ds_corner_cloud_pub_ptr_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/front_end/cur_corner_ds", 100);
    current_ds_surface_cloud_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/front_end/cur_surf_ds", 100);
    corner_from_map_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/front_end/corner_from_map", 100);
    surface_from_map_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/front_end/surf_from_map", 100);
    global_corner_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_corner_map", 10);
    global_surf_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_surface_map", 10);
    global_map_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_raw_map", 10);
}

void FrontEndFlow::saveMap(const msg_interface::srv::SaveMap::Request::SharedPtr req, 
                           const msg_interface::srv::SaveMap::Response::SharedPtr res){
    pcl::PointCloud<PointType>::Ptr corner_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surf_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr global_cloud(new pcl::PointCloud<PointType>());
    front_end_ptr_->saveGlobalMap(req, corner_cloud, surf_cloud, global_cloud);
    sensor_msgs::msg::PointCloud2 corner_map_ros, surf_map_ros, global_map;
    pcl::toROSMsg(*corner_cloud, corner_map_ros);
    pcl::toROSMsg(*surf_cloud, surf_map_ros);
    pcl::toROSMsg(*global_cloud, global_map);
    corner_map_ros.header.frame_id = map_frame_;
    surf_map_ros.header.frame_id = map_frame_;
    global_map.header.frame_id = map_frame_;
    global_corner_pub_ptr_->publish(corner_map_ros);
    global_surf_pub_ptr_->publish(surf_map_ros);
    global_map_pub_ptr_->publish(global_map);
    res->success = true;
    LOG(INFO) << "Publish global map ro rviz ... \n";
}


void FrontEndFlow::gnssOdomHandler(const nav_msgs::msg::Odometry::SharedPtr gnss_data){
    std::lock_guard<std::mutex> lock(gnss_lock_);
    if(!front_end_ptr_->isInited())
        gnss_deque_.push_back(*gnss_data);
}


void FrontEndFlow::optKeyFramesHandler(const sensor_msgs::msg::PointCloud2::SharedPtr opt_key_frames){
    // 设置关键帧的pose,以及关键帧对应的角点、平面点集合 
    cloud_info_lock_.lock();
    front_end_ptr_->setKeyFrame(*opt_key_frames);
    cloud_info_lock_.unlock();
}

bool FrontEndFlow::initPose(){
    if(front_end_ptr_->isInited()) return true;
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
    }
    front_end_ptr_->setInitPose(init_pose);  
    return true;
}

void FrontEndFlow::cloudInfoHandler(const msg_interface::msg::CloudInfo::SharedPtr cloud_info){
    std::lock_guard<std::mutex> lock(cloud_info_lock_);
    // 得到当前cloud info数据
    current_cloud_info_ = *cloud_info;
    front_end_ptr_->setCloudInfo(current_cloud_info_);
    
    // 系统初始化
    if(!initPose()) return;

    auto t1 = std::chrono::system_clock::now();
    // 前端里程计匹配求解pose 
    updateLaserOdometry();    
    // 发布前端激光里程计    
    publishData();
    auto t2 = std::chrono::system_clock::now();
    LOG(INFO) << "updateLaserOdometry cost time : " << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() << " ms." << std::endl;
}

void FrontEndFlow::updateLaserOdometry(){
    // 设置位姿初始值
    front_end_ptr_->updateInitialGuess();
    
    // 最近邻搜索找到当前帧附近的关键帧组成local map，用于匹配
    front_end_ptr_->extractSurroundingKeyFrames();

    // 将当前帧角点和平面点降采样
    front_end_ptr_->downsampleCurrentScan(current_cloud_info_);
    
    // scan to map优化得到激光里程计 打开openmp平均耗时约20～30ms左右，有时到50～80ms， 不开openMP的话超过几百毫秒
    front_end_ptr_->scan2MapOptimization();
}

void FrontEndFlow::publishData(){
    // pub laser pose to back end
    msg_interface::msg::KeyFramePose current_frame_pose;
    front_end_ptr_->getFramePose(current_frame_pose);
    laser_pose_pub_ptr_->publish(current_frame_pose);
    
    // publish current local map    
    sensor_msgs::msg::PointCloud2 corner_from_map;
    sensor_msgs::msg::PointCloud2 surf_from_map;
    front_end_ptr_->getFeatureFromMap(corner_from_map, surf_from_map);
    corner_from_map.header.frame_id = map_frame_;
    surf_from_map.header.frame_id = map_frame_;
    corner_from_map_pub_ptr_->publish(corner_from_map);
    surface_from_map_pub_ptr_->publish(surf_from_map); 

    // pub laser odom (base frame)
    nav_msgs::msg::Odometry laser_odom;
    laser_odom.header.stamp = current_cloud_info_.header.stamp;
    laser_odom.header.frame_id = odom_frame_;       // odom
    laser_odom.child_frame_id = "laser_odom";    // laser odom
    transToOdom(current_frame_pose, laser_odom);
    laser_odom_pub_ptr_->publish(laser_odom);

    // pub base odom
    nav_msgs::msg::Odometry base_odom;
    base_odom.header.stamp = laser_odom.header.stamp;
    base_odom.header.frame_id = odom_frame_;              // odom
    base_odom.child_frame_id  = "laser_base_odom";        // baselink odom
    front_end_ptr_->getBaseOdom(base_odom);
    base_odom_pub_ptr_->publish(base_odom);

    // 保存当前帧用于构建局部地图
    if(current_frame_pose.is_key_frame)
        front_end_ptr_->saveKeyFrame();
}
}// namespace truck_slam