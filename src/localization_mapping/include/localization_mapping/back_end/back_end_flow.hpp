/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-23 11:51:33
 */
#ifndef LOCALIZATION_MAPPING_BACK_END_BACK_END_FLOW_HPP_
#define LOCALIZATION_MAPPING_BACK_END_BACK_END_FLOW_HPP_

#include <deque>
#include <mutex>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "glog/logging.h"

#include "localization_mapping/back_end/back_end.hpp"
#include "localization_mapping/utils/sensor_data/costom_point_type.hpp"
#include "localization_mapping/utils/tools/tools.hpp"
#include "msg_interface/msg/loop_pose.hpp"
#include "msg_interface/msg/key_frame.hpp"
#include "msg_interface/msg/key_frame_pose.hpp"

namespace truck_slam{
class BackEndFlow : public rclcpp::Node{
public:
    BackEndFlow(const std::string &node_name, const rclcpp::NodeOptions &options);
    void paramsInit();
    void resetParametes();
    void subAndPubInit();

    void GNSSHandler(const nav_msgs::msg::Odometry::SharedPtr gnss_data);
    void LoopClosureHandler(const msg_interface::msg::LoopPose::SharedPtr loop_pose);
    void laserFrameHandler(const msg_interface::msg::KeyFramePose::SharedPtr laser_frame);
    void publishData();
    void insertLoopPose();
    bool findValidGNSSPose(const msg_interface::msg::KeyFramePose &current_laser_odom, nav_msgs::msg::Odometry &current_gnss_odom);
    void publishLaserOdom(nav_msgs::msg::Odometry &odom);

private:
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group1_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group2_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group3_;

    // back end ptr
    std::shared_ptr<BackEnd> back_end_ptr_;
    // topic and frame name
    std::string laser_odom_topic_;
    std::string gnss_odom_topic_;
    std::string loop_pose_topic_;
    std::string corner_surf_frame_topic_;
    std::string odom_opt_topic_;
    std::string odom_incre_topic_;
    std::string key_frame_topic_;
    std::string odometry_frame_;
    std::string map_frame_;
    std::string laser_frame_;
    // gnss settings
    Eigen::MatrixXd pose_covariance_;   
    double gnss_cov_threshold_;
    double pose_cov_threshold_;
    bool use_gnss_elevation_;
    bool last_incre_odom_pub_flag_;
    // subscribers
    rclcpp::Subscription<msg_interface::msg::KeyFramePose>::SharedPtr laser_odom_sub_ptr_;      // 订阅当前帧角点、平面点，以及前端匹配得到的pose
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gnss_odom_sub_ptr_;
    rclcpp::Subscription<msg_interface::msg::LoopPose>::SharedPtr loop_pose_sub_ptr_;
    // publishers
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_odom_to_laser_ptr_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_opt_pub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr key_frame_pub_ptr_;          // 发布关键帧位置、姿态（欧拉角度）、关键帧索引、关键帧的角点、平面点
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_ptr_;

    // data buff
    std::deque<msg_interface::msg::KeyFrame> laser_odom_buff_;
    std::deque<nav_msgs::msg::Odometry> gnss_odom_buff_;
    std::deque<msg_interface::msg::LoopPose> loop_pose_buff_;

    // current data
    nav_msgs::msg::Odometry current_gnss_odom_;
    msg_interface::msg::KeyFramePose current_laser_odom_;
    std_msgs::msg::Header current_laser_header_;
    nav_msgs::msg::Odometry laser_odom_opt_;
    nav_msgs::msg::Odometry laser_odom_opt_incre_;
    msg_interface::msg::LoopPose current_loop_pose_;
    Eigen::Affine3f incre_odom_affine_;
    Eigen::Affine3f incremental_odometry_affine_front_;
    Eigen::Affine3f incremental_odometry_affine_back_;

    // lock
    std::mutex gnss_lock_;
    std::mutex loop_pose_lock_;
    std::mutex laser_frame_lock_;
};
}
#endif