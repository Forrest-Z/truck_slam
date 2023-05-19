/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-08 13:41:54
 */

#ifndef LOCALIZATION_MAPPING_DATA_PRETREAT_FLOW_HPP_
#define LOCALIZATION_MAPPING_DATA_PRETREAT_FLOW_HPP_


#include <iostream>
#include <deque>
#include <vector>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv4/opencv2/opencv.hpp>
#include <pcl/filters/filter.h>
#include "localization_mapping/data_pretreat/data_pretreat.hpp"
#include "localization_mapping/utils/model/filter/box_filter.hpp"
#include "localization_mapping/utils/tools/tools.hpp"
#include "localization_mapping/utils/sensor_data/costom_point_type.hpp"

#include "msg_interface/msg/cloud_info.hpp"

#define DEBUG


namespace truck_slam{

class DataPretreatFlow : public rclcpp::Node{

public:   
    DataPretreatFlow(const std::string& node_name, const rclcpp::NodeOptions &options);
    void paramsInit();
    void resetParametes();
    void subAndPubInit();
    
    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_data);
    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_data);
    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometry_msg);
    bool loadData(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_data);
    bool findValidData();
    void pointCloudToPointXYZRIT(sensor_msgs::msg::PointCloud2 &cloud_in,
                                pcl::PointCloud<PointXYZIRT>::Ptr &cloud_out); 
    void projectPointCloud();
    void deskewPointCloud();
    void cloudExtraction();
    void featureExtraction();
    void publishData();

private:
    std::shared_ptr<DataPretreat> data_pretreat_ptr_;

    // range mat
    cv::Mat range_mat_;

    // call back group
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub1_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub2_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub3_;

    // sub and pub
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_sub_ptr_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_imu_sub_ptr_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr extracted_cloud_pub_ptr_;
    rclcpp::Publisher<msg_interface::msg::CloudInfo>::SharedPtr cloud_info_pub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corner_cloud_pub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_cloud_pub_ptr_;
    // ros variables
    std::string imuTopic_;
    std::string odomTopic_;
    std::string pointCloudTopic_;
    std::string extractedCloudTopic_;
    std::string cloudInfoTopic_;
    std::string cornerTopic_;
    std::string surfaceTopic_;
    std::string sensor_str_;

    std_msgs::msg::Header current_cloud_header_;

    //box filter
    std::shared_ptr<BoxFilter<PointXYZIRT>> box_filter_ptr_;
    // sensor settings
    int N_SCAN_;
    int Horizon_SCAN_;

    // data buff
    std::deque<sensor_msgs::msg::Imu> raw_imu_deque_;
    std::deque<sensor_msgs::msg::PointCloud2> raw_cloud_deque_;
    std::deque<nav_msgs::msg::Odometry> odom_deque_;
    
    // timestamp of current cloud data
    double current_cloud_start_timestamp_;
    double current_cloud_end_timestamp_;
    
    // current data
    sensor_msgs::msg::Imu::SharedPtr current_raw_imu_data_;
    pcl::PointCloud<PointXYZIRT>::Ptr current_cloud_data_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_data_;
    msg_interface::msg::CloudInfo current_cloud_info_;
    pcl::PointCloud<PointType>::Ptr full_point_cloud_;
    pcl::PointCloud<PointType>::Ptr extracted_cloud_;
    pcl::PointCloud<PointType>::Ptr corner_cloud_;
    pcl::PointCloud<PointType>::Ptr surface_cloud_;

    std::mutex imu_mutex_;
    std::mutex odom_mutex_;
    std::mutex cloud_mutex_;
};

} // truck_slam

#endif