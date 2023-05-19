/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-13 09:33:08
 */
#ifndef LOCALIZATION_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_
#define LOCALIZATION_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_

#include <deque>
#include <mutex>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "localization_mapping/front_end/front_end.hpp"
#include "localization_mapping/utils/sensor_data/costom_point_type.hpp"
#include "msg_interface/msg/cloud_info.hpp"
#include "msg_interface/msg/key_frame.hpp"
#include "msg_interface/msg/key_frame_pose.hpp"
#include "msg_interface/srv/save_map.hpp"

namespace truck_slam{
class FrontEndFlow : public rclcpp::Node{
public:
    FrontEndFlow(const std::string node_name, const rclcpp::NodeOptions &option);
    void paramsInit();
    void resetParametes();
    void subAndPubInit();
    bool initPose();
    void cloudInfoHandler(const msg_interface::msg::CloudInfo::SharedPtr cloud_info);
    void optKeyFramesHandler(const sensor_msgs::msg::PointCloud2::SharedPtr opt_key_frames);
    void gnssOdomHandler(const nav_msgs::msg::Odometry::SharedPtr gnss_data);
    bool findValidData();
    void updateLaserOdometry();
    void publishData();
    void saveMap(const msg_interface::srv::SaveMap_Request::SharedPtr req, const msg_interface::srv::SaveMap_Response::SharedPtr res);

private:
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group1_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group2_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group3_;

    std::shared_ptr<FrontEnd> front_end_ptr_;

    // save map service
    rclcpp::Service<msg_interface::srv::SaveMap>::SharedPtr service_;   

    // subscribers and publishers
    rclcpp::Subscription<msg_interface::msg::CloudInfo>::SharedPtr cloud_info_sub_ptr_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr opt_key_frames_sub_ptr_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gnss_sub_ptr_;
    rclcpp::Publisher<msg_interface::msg::KeyFramePose>::SharedPtr laser_pose_pub_ptr_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr laser_odom_pub_ptr_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr base_odom_pub_ptr_;

    std::string cloud_info_topic_;
    std::string gnss_topic_;
    std::string opt_key_frame_topic_;
    std::string laser_odom_topic_;
    std::string base_odom_topic_;
    std::string laser_frame_pose_topic_;
    std::string odom_frame_;
    std::string map_frame_;
    std::string base_frame_;
    std::string init_method_;

    // data buff
    std::deque<msg_interface::msg::CloudInfo> cloud_info_deque_;
    std::deque<nav_msgs::msg::Odometry> gnss_deque_;

    //current data
    msg_interface::msg::CloudInfo current_cloud_info_;

    std::mutex cloud_info_lock_;
    std::mutex opt_keyframe_lock_;
    std::mutex global_updated_lock_;
    std::mutex gnss_lock_;

    bool use_imu_heading_initial_;
    // debug
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_surf_pub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_corner_pub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corner_from_map_pub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_from_map_pub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_ds_corner_cloud_pub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_ds_surface_cloud_pub_ptr_;
};
}
#endif