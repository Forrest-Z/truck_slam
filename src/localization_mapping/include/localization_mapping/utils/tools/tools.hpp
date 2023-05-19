/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-11 22:58:01
 */
#pragma once
#ifndef LOCALIZATION_MAPPING_TOOLS_HPP_
#define LOCALIZATION_MAPPING_TOOLS_HPP_

#include <fstream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h> 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include "localization_mapping/utils/sensor_data/costom_point_type.hpp"
#include "msg_interface/msg/key_frame_pose.hpp"
#include <omp.h>


namespace truck_slam{
    
double getRosTime(const builtin_interfaces::msg::Time &stamp);
template<typename T>
void imuRPY2rosRPY(const sensor_msgs::msg::Imu &thisImuMsg, T &rosRoll, T &rosPitch, T &rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation(thisImuMsg.orientation.x , thisImuMsg.orientation.y ,
                                thisImuMsg.orientation.z, thisImuMsg.orientation.w);
    tf2::Matrix3x3(orientation).getRPY(imuRoll , imuPitch, imuYaw);
    rosRoll = imuRoll;
    rosPitch = imuPitch;
    rosYaw = imuYaw;
}
template<typename T>
void imuAngular2rosAngular(const sensor_msgs::msg::Imu &thisImuMsg, T &angular_x, T &angular_y, T &angular_z)
{
    angular_x = thisImuMsg.angular_velocity.x;
    angular_y = thisImuMsg.angular_velocity.y;
    angular_z = thisImuMsg.angular_velocity.z;
}
float pointDistance(PointType p1, PointType p2);
float pointDistance(pcl::PointXYZI p1, pcl::PointXYZI p2);
float pointRange(PointType p);
void savePose(std::ofstream &ofs, const Eigen::Matrix4f &pose);
pcl::PointCloud<pcl::PointXYZI>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointXYZI> cloudIn, PointXYZIRPYT* transforIn);
void transToPointPose(const msg_interface::msg::KeyFramePose &pose_data, PointTypePose &point_pose);
gtsam::Pose3 transToPose3(const nav_msgs::msg::Odometry &pose_data);
gtsam::Pose3 transToPose3(const msg_interface::msg::KeyFramePose &pose_data);
gtsam::Pose3 transToPose3(const PointTypePose &pose_data);
void transToOdom(const gtsam::Pose3 &pose_data, nav_msgs::msg::Odometry &odom_data);
void transToOdom(const msg_interface::msg::KeyFramePose &pose_data, geometry_msgs::msg::TransformStamped &odom_data);
void transToOdom(const msg_interface::msg::KeyFramePose &pose_data, nav_msgs::msg::Odometry &odom_data);
void transToArray(const Eigen::Matrix4f &transform, float* data);
}


#endif