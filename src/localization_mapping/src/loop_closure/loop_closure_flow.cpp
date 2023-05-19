/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-03-06 10:44:26
 */
#include "localization_mapping/loop_closure/loop_closure_flow.hpp"

namespace truck_slam{    
LoopClosureFlow::LoopClosureFlow(const std::string &node_name): Node(node_name){
    paramsInit();
    resetParametes();
    subAndPubInit();
    LOG(INFO) << "*************** Loop Closure node Inited ***************\n";
}

void LoopClosureFlow::paramsInit(){
    // LOG(INFO) <<  "Parameters declare ... \n";
    YAML::Node config_node = YAML::LoadFile(WORK_SPACE_PATH + "/config/config.yaml");
    key_frame_topic_ = config_node["topic"]["keyFrameTopic"].as<std::string>();
    loop_pose_topic_ = config_node["topic"]["loopPoseTopic"].as<std::string>();
    use_loop_ = config_node["loop_closure"]["useLoopClosure"].as<bool>();
}

void LoopClosureFlow::resetParametes(){
    // LOG(INFO) <<  "Parameters and variables reset ... \n";
    cloud_key_frames_3d_.reset(new pcl::PointCloud<PointType>());
    cloud_key_frames_6d_.reset(new pcl::PointCloud<PointTypePose>());
    cur_key_frame_cloud_.reset(new pcl::PointCloud<PointType>());
    pre_key_frame_cloud_.reset(new pcl::PointCloud<PointType>());
    loop_closure_ptr_ = std::make_shared<LoopClosure>();
}

void LoopClosureFlow::subAndPubInit(){
    // LOG(INFO) <<  "Subscribers and Publishers init ... \n";
    loop_pose_pub_ptr_ = this->create_publisher<LoopPose>(loop_pose_topic_, 1000);
    loop_local_map_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cur_loop_local_map", 10);
    loop_cur_scan_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cur_loop_scan", 10);
    opt_key_frames_sub_ptr_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(key_frame_topic_, 1000, std::bind(&LoopClosureFlow::keyFramesHandler, this, std::placeholders::_1));
}

void LoopClosureFlow::keyFramesHandler(const sensor_msgs::msg::PointCloud2::SharedPtr key_frames){
    if(!use_loop_) return;
    
    // 设置关键帧的pose,以及关键帧对应的角点、平面点集合
    loop_closure_ptr_->setKeyFrames(*key_frames);

    cur_key_frame_cloud_.reset(new pcl::PointCloud<PointType>());
    pre_key_frame_cloud_.reset(new pcl::PointCloud<PointType>());
    if(loop_closure_ptr_->Detect(current_loop_pose_, cur_key_frame_cloud_, pre_key_frame_cloud_)){
        LOG(INFO) << "find loop pose and publish it to back end ...\n";
        loop_pose_pub_ptr_->publish(current_loop_pose_);
    }
    
    // debug
    sensor_msgs::msg::PointCloud2 cur_local_map, cur_scan;
    pcl::toROSMsg(*cur_key_frame_cloud_, cur_scan);
    pcl::toROSMsg(*pre_key_frame_cloud_, cur_local_map); 
    cur_scan.header.frame_id = "odom";
    cur_local_map.header.frame_id = "odom";
    loop_local_map_pub_ptr_->publish(cur_local_map);
    loop_cur_scan_pub_ptr_->publish(cur_scan);
}
}