/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-03-06 10:16:25
 */
#ifndef LOCALIZATION_MAPPING_LOOP_CLOSURE_LOOP_CLOSURE_FLOW_HPP_
#define LOCALIZATION_MAPPING_LOOP_CLOSURE_LOOP_CLOSURE_FLOW_HPP_

#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "localization_mapping/loop_closure/loop_closure.hpp"
#include "localization_mapping/utils/sensor_data/costom_point_type.hpp"
#include "msg_interface/msg/key_frame.hpp"
#include "msg_interface/msg/loop_pose.hpp"

namespace truck_slam{
class LoopClosureFlow : public rclcpp::Node{
public:
    using KeyFrame = msg_interface::msg::KeyFrame;
    using LoopPose = msg_interface::msg::LoopPose;
    
    LoopClosureFlow(const std::string &node_name);
    void paramsInit();
    void resetParametes();
    void subAndPubInit();
    void keyFramesHandler(const sensor_msgs::msg::PointCloud2::SharedPtr key_frame);

private:
    // loop closure
    std::shared_ptr<LoopClosure> loop_closure_ptr_;

    // current loop detect pub
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loop_local_map_pub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loop_cur_scan_pub_ptr_;

    // key frames subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr opt_key_frames_sub_ptr_;
    rclcpp::Publisher<LoopPose>::SharedPtr loop_pose_pub_ptr_;
    std::string key_frame_topic_;
    std::string loop_pose_topic_;
    bool use_loop_;

    // data buff
    std::deque<KeyFrame> key_frame_buff_;
    KeyFrame current_key_frame_;
    pcl::PointCloud<PointType>::Ptr cloud_key_frames_3d_;
    pcl::PointCloud<PointTypePose>::Ptr cloud_key_frames_6d_;
    std::vector<pcl::PointCloud<PointType>::Ptr> corner_cloud_key_frames_;
    std::vector<pcl::PointCloud<PointType>::Ptr> surface_cloud_key_frames_;
    pcl::PointCloud<PointType>::Ptr cur_key_frame_cloud_;
    pcl::PointCloud<PointType>::Ptr pre_key_frame_cloud_;
    LoopPose current_loop_pose_;
};
}
#endif