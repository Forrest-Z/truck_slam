/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-23 11:51:13
 */
#include "localization_mapping/back_end/back_end_flow.hpp"

namespace truck_slam{
BackEndFlow::BackEndFlow(const std::string &node_name, const rclcpp::NodeOptions &options)
                        : Node(node_name, options){
    paramsInit();
    resetParametes();
    subAndPubInit();
    LOG(INFO) << "*************** Back End node Inited ***************\n";

}

void BackEndFlow::paramsInit(){
    // LOG(INFO) <<  "Parameters declare ... \n";
    YAML::Node config_node = YAML::LoadFile(WORK_SPACE_PATH + "/config/config.yaml");
    laser_odom_topic_ = config_node["topic"]["laserFramePoseTopic"].as<std::string>();
    key_frame_topic_ = config_node["topic"]["keyFrameTopic"].as<std::string>();
    odom_opt_topic_ = config_node["topic"]["optOdomTopic"].as<std::string>();
    gnss_odom_topic_ = config_node["topic"]["gnssPoseTopic"].as<std::string>();
    loop_pose_topic_ = config_node["topic"]["loopPoseTopic"].as<std::string>();
    odometry_frame_ = config_node["frame"]["odometryFrame"].as<std::string>();
    map_frame_ = config_node["frame"]["mapFrame"].as<std::string>();
    laser_frame_ = config_node["frame"]["laserFrame"].as<std::string>();
    gnss_cov_threshold_ = config_node["gnss"]["gnssCovThreshold"].as<double>();
    pose_cov_threshold_ = config_node["gnss"]["poseCovThreshold"].as<double>();
    use_gnss_elevation_ = config_node["gnss"]["useGnssElevation"].as<bool>();
}

void BackEndFlow::resetParametes(){
    // LOG(INFO) <<  "Parameters and variables reset ... \n";
    back_end_ptr_ = std::make_shared<BackEnd>();
    last_incre_odom_pub_flag_ = false;
}

void BackEndFlow::subAndPubInit(){
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

    // 订阅话题：激光里程计，gnss数据，回环检测的结果
    laser_odom_sub_ptr_ = this->create_subscription<msg_interface::msg::KeyFramePose>(laser_odom_topic_, 100, std::bind(&BackEndFlow::laserFrameHandler, this, std::placeholders::_1), sub_opt1);
    gnss_odom_sub_ptr_ = this->create_subscription<nav_msgs::msg::Odometry>(gnss_odom_topic_, 1000, std::bind(&BackEndFlow::GNSSHandler, this, std::placeholders::_1), sub_opt2);
    loop_pose_sub_ptr_ = this->create_subscription<msg_interface::msg::LoopPose>(loop_pose_topic_, 1000, std::bind(&BackEndFlow::LoopClosureHandler, this, std::placeholders::_1), sub_opt3);
    tf_odom_to_laser_ptr_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // 发布话题：关键帧点云、关键帧3d&6d pose、优化后的位姿
    /* “mapping/odometry_incremental”和“mapping/odometry”的区别：
       “mapping/odometry”： 是经过前端或者后端优化后得到最终激光里程计pose
       “mapping/odometry_incremental”： 先求出相邻两激光帧（不一定是关键帧）的相对pose，其中第k帧的pose是前端匹配得到的，k-1帧的则是联合优化得到的
                                       再将上一帧的pose乘以相对pose得到当前帧的pose，最后和当前帧时刻的imu原始数据进行加权限制以下roll和pitch，
                                       最终得到当前帧的pose
     */
    odom_opt_pub_ptr_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_opt_topic_, 1000);  
    key_frame_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(key_frame_topic_, 1000);
    global_path_pub_ptr_ = this->create_publisher<nav_msgs::msg::Path>("/back_end/global_path", 100);
}

void BackEndFlow::GNSSHandler(const nav_msgs::msg::Odometry::SharedPtr gnss_data){
    std::lock_guard<std::mutex> lock(gnss_lock_);
    LOG(INFO) << "gnss_data coming...\n";  
    gnss_odom_buff_.push_back(*gnss_data);
}


void BackEndFlow::LoopClosureHandler(const msg_interface::msg::LoopPose::SharedPtr loop_pose){
    std::lock_guard<std::mutex> lock(loop_pose_lock_);
    loop_pose_buff_.push_back(*loop_pose);
    while (loop_pose_buff_.size() > 5)
        loop_pose_buff_.pop_front();
}

void BackEndFlow::laserFrameHandler(const msg_interface::msg::KeyFramePose::SharedPtr laser_frame){
    // get crrent laser frame
    current_laser_odom_ = *laser_frame;
    current_laser_header_.stamp = rclcpp::Time(current_laser_odom_.time * 1e9);
    current_laser_header_.frame_id = odometry_frame_;

    if(current_laser_odom_.is_key_frame){
        // add loop pose
        insertLoopPose();

        // check gnss pose add it 
        if(findValidGNSSPose(current_laser_odom_, current_gnss_odom_)){
            back_end_ptr_->addGNSSFactor(current_gnss_odom_);
            LOG(INFO) << "backend addGNSSFactor...\n";  
        }

        // add laser odom factor
        back_end_ptr_->addLaserOdomFactor(current_laser_odom_);   

        // update back end pose
        auto t2 = std::chrono::system_clock::now(); 
        back_end_ptr_->Update(current_laser_odom_, pose_covariance_);
        auto t3 = std::chrono::system_clock::now(); 
        LOG(INFO) << "backend Update cost time : " << std::chrono::duration_cast<std::chrono::milliseconds>(t3-t2).count() << "ms. \n";  
    }
    // publish odometry
    publishData();
}

void BackEndFlow::insertLoopPose(){
    while(!loop_pose_buff_.empty()){
        back_end_ptr_->addLoopFactor(loop_pose_buff_.front());
        loop_pose_buff_.pop_front();
        LOG(INFO) <<  "back end insert loop pose ... \n";
    }
}

bool BackEndFlow::findValidGNSSPose(const msg_interface::msg::KeyFramePose &current_laser_odom,
                                    nav_msgs::msg::Odometry &current_gnss_odom){
    if(gnss_odom_buff_.empty())
        return false;
    // 如果没有关键帧，或者首尾关键帧距离小于5m，不添加gps因子
    if(back_end_ptr_->keyFrameEmpty())
        return false;
    // else if(pointDistance(cloud_key_frame_3d_->front(), cloud_key_frame_3d_->back()) < 2.0)
    //     return false;

    // 位姿协方差很小，没必要加入gnss数据进行校正
    // if(pose_covariance_(3,3) < pose_cov_threshold_ && pose_covariance_(4,4) < pose_cov_threshold_)
    //     return false;

    // 保留的近200ms内的gnss数据
    static PointType last_gnss_pose;
    double laser_odom_time = current_laser_odom.time;
    while(!gnss_odom_buff_.empty()){
        double gnss_odom_time = rclcpp::Time(gnss_odom_buff_.front().header.stamp).seconds();
        // LOG(INFO) << "gnss odom time : " << std::to_string(gnss_odom_time) << " , laser_odom_time = " << std::to_string(laser_odom_time) << std::endl;
        if(gnss_odom_time < laser_odom_time - 0.1){
            // LOG(INFO) << "gnss odom time < laser odom time - 0.2, gnss data deque pop_front .. \n";
            gnss_odom_buff_.pop_front();
        }else if(gnss_odom_time > laser_odom_time + 0.1){
            // LOG(INFO) << "gnss odom time > laser odom time + 0.2, gnss data is too late, can not be used .. \n";
            break;
        }else{

            nav_msgs::msg::Odometry current_gnss_data = gnss_odom_buff_.front();
            gnss_odom_buff_.pop_front();
            
            float noise_x = current_gnss_data.pose.covariance[0];
            float noise_y = current_gnss_data.pose.covariance[7];
            float noise_z = current_gnss_data.pose.covariance[14];
            if(noise_x > gnss_cov_threshold_ || noise_y > gnss_cov_threshold_)
                continue;

            float gnss_x = current_gnss_data.pose.pose.position.x;
            float gnss_y = current_gnss_data.pose.pose.position.y;
            float gnss_z = current_gnss_data.pose.pose.position.z;

            // 如果gnss高度方向观测不可靠
            if (!use_gnss_elevation_){
                gnss_z = current_laser_odom.z;
                current_gnss_data.pose.pose.position.z = current_laser_odom.z;
                current_gnss_data.pose.covariance[14] = 0.01;
            }

            // gnss not properly initialized (0,0,0)
            if (abs(gnss_x) < 1e-6 && abs(gnss_y) < 1e-6)
                continue;

            // // 保留前2m内的gnss数据
            // PointType current_gnss_pose;
            // current_gnss_pose.x = gnss_x;
            // current_gnss_pose.y = gnss_y;
            // current_gnss_pose.z = gnss_z;
            // if (pointDistance(current_gnss_pose, last_gnss_pose) < 5.0)
            //     continue;
            // else
            //     last_gnss_pose = current_gnss_pose;
            
            current_gnss_odom = current_gnss_data; 
            return true;
        }
    }

    return false;
}

void BackEndFlow::publishLaserOdom(nav_msgs::msg::Odometry &odom){
    odom.header = current_laser_header_;
    odom.child_frame_id = "odom_mapping";
    transToOdom(current_laser_odom_, odom);
    odom_opt_pub_ptr_->publish(odom);
}

void BackEndFlow::publishData(){
    // frame_id : odom, child_frame_id : odom_mapping 
    publishLaserOdom(laser_odom_opt_);

    // 发布当前关键帧集合
    sensor_msgs::msg::PointCloud2 key_frames;
    back_end_ptr_->getKeyFrames(key_frames);
    key_frame_pub_ptr_->publish(key_frames);

    // pub global path
    nav_msgs::msg::Path path;
    back_end_ptr_->getGlobalPath(path);
    path.header = current_laser_header_;
    global_path_pub_ptr_->publish(path);                              

    // odom to lidar tf 
    geometry_msgs::msg::TransformStamped odom_to_laser;
    odom_to_laser.header = current_laser_header_;
    odom_to_laser.child_frame_id  = laser_frame_;
    transToOdom(current_laser_odom_, odom_to_laser);
    tf_odom_to_laser_ptr_->sendTransform(odom_to_laser);
}
}