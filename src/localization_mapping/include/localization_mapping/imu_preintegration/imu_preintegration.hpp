/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-03-06 19:32:16
 */
#ifndef LOCALIZATION_MAPPING_IMU_PREINTEGRATION_HPP_
#define LOCALIZATION_MAPPING_IMU_PREINTEGRATION_HPP_

#include <chrono>
#include <mutex>
#include <deque>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "localization_mapping/global_defination/global_defination.h"
#include "localization_mapping/utils/tools/tools.hpp"
#include "glog/logging.h"

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::R; 


namespace truck_slam{
class IMUPreIntegration : public rclcpp::Node{
public:    
    IMUPreIntegration(const std::string &node_name , const rclcpp::NodeOptions &options);

    void loadParams();
    void initParams();
    void subAndPubInit();
    void resetOptimization();
    void resetParams();
    void optLaserOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    bool failureDetection(const gtsam::Vector3& vel_cur, const gtsam::imuBias::ConstantBias& bias_cur);
    void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr imu_raw);
private:
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group1_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group2_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_raw_sub_ptr_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opt_laser_odom_sub_ptr_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_pub_ptr_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr baselink_odom_pub_ptr_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_odom_to_imu_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_odom_to_baselink_;
    tf2::Transform lidar_to_baselink_;
    tf2::Transform imu_to_baselink_;

    std::string opt_laser_odom_topic_;
    std::string imu_raw_topic_;
    std::string imu_odom_topic_;
    std::string baselink_odom_topic_;
    std::string odometry_frame_;
    std::string laser_frame_;
    std::string base_link_frame_;
    std::string imu_frame_;

    std::mutex imu_mtx_;
    std::mutex laser_odom_mtx_;
    std::deque<sensor_msgs::msg::Imu> imu_que_opt_;
    std::deque<sensor_msgs::msg::Imu> imu_que_imu_;

    gtsam::ISAM2 optimizer_;
    gtsam::NonlinearFactorGraph graph_factor_;
    gtsam::Values graph_values_;
    gtsam::PreintegratedImuMeasurements *imu_integrator_opt_;
    gtsam::PreintegratedImuMeasurements *imu_integrator_imu_;
    gtsam::Pose3 prev_pose_;
    gtsam::Vector3 prev_vel_;
    gtsam::NavState prev_state_;
    gtsam::imuBias::ConstantBias prev_bias_;
    gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr prior_vel_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr prior_bias_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr correction_noise_;
    gtsam::Vector noise_model_between_bias_;

    gtsam::NavState prev_state_odom_;
    gtsam::imuBias::ConstantBias prev_bias_odom_;
    gtsam::Pose3 lidar_to_imu_;
    std::vector<float> imu_to_lidar_vec_;
    std::vector<float> baselink_to_imu_vec_;

    float imu_acc_noise_;
    float imu_gyr_noise_;
    float imu_acc_biasN_;
    float imu_gyr_biasN_;
    float imu_gravity_;
    int imu_freq_;
    int imu_type_ = 0;


    const double delta_t_ = 0.0;
    bool done_first_opt_;
    bool system_inited_;

    int counter_ = 0;

    double last_imu_time_opt_;
    double last_imu_time_;
};
}

#endif