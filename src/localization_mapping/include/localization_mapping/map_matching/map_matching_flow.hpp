/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-04-13 18:47:26
 */
#ifndef LOCALIZATION_MAPPING_MAP_MATCHING_FLOW_HPP_
#define LOCALIZATION_MAPPING_MAP_MATCHING_FLOW_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/impl/angles.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include "localization_mapping/map_matching/map_matching.hpp"
#include "localization_mapping/utils/tools/tools.hpp"
#include "localization_mapping/utils/model/scan_context/scan_context_manager.hpp"
#include "msg_interface/msg/cloud_info.hpp"

namespace truck_slam{
class MapMatchingFlow : public rclcpp::Node{
public:
    MapMatchingFlow(const std::string node_name, const rclcpp::NodeOptions &option);
    void paramsInit();
    void resetParametes();
    void subAndPubInit();
    bool initPose();
    bool setScanContextPose(const msg_interface::msg::CloudInfo &cloud_info, float* pose);
    void cloudInfoHandler(const msg_interface::msg::CloudInfo::SharedPtr cloud_info);
    void gnssOdomHandler(const nav_msgs::msg::Odometry::SharedPtr gnss_data);
    void rvizInitHandler(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr rviz_init_pose);
    void updateLaserOdometry();
    void publishData();

private:
    std::shared_ptr<MapMatching> map_matching_ptr_;
    std::shared_ptr<ScanContextManager> scan_context_manager_ptr_;

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group1_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group2_;\
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group3_;

    // subscribers and publishers
    rclcpp::Subscription<msg_interface::msg::CloudInfo>::SharedPtr cloud_info_sub_ptr_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gnss_sub_ptr_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rviz_init_pose_sub_ptr_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr laser_odom_pub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corner_local_map_pub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surf_local_map_pub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corner_global_map_pub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surf_global_map_pub_ptr_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_odom_to_laser_ptr_;
    std::string gnss_topic_;
    std::string cloud_info_topic_;
    std::string laser_odom_topic_;
    std::string map_frame_;
    std::string laser_frame_;
    std::string init_method_;
    bool use_imu_heading_initial_;
    bool is_update_ ;
    bool rviz_init_pose_arived_;

    // rviz init pose
    float rviz_init_pose_[6];
    // data buff
    std::deque<msg_interface::msg::CloudInfo> cloud_info_deque_;
    std::deque<nav_msgs::msg::Odometry> gnss_deque_;
    //current data
    msg_interface::msg::CloudInfo current_cloud_info_;
    std::mutex cloud_info_lock_;
    std::mutex gnss_lock_;
};
}
#endif