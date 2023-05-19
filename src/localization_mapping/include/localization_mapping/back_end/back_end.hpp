/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-23 11:51:40
 */
#ifndef LOCALIZATION_MAPPING_BACK_END_BACK_END_HPP_
#define LOCALIZATION_MAPPING_BACK_END_BACK_END_HPP_

#include <cmath>
#include <deque>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <glog/logging.h>
#include "msg_interface/msg/loop_pose.hpp"
#include "msg_interface/msg/key_frame.hpp"
#include "msg_interface/msg/key_frame_pose.hpp"
#include "localization_mapping/utils/sensor_data/costom_point_type.hpp"
#include "localization_mapping/global_defination/global_defination.h"
#include "localization_mapping/utils/tools/tools.hpp"

namespace truck_slam{
class BackEnd{
public:
    using PointTypePose = PointXYZIRPYT;
    using PointType = pcl::PointXYZI;

    BackEnd();
    void paramsLoad();
    void allocateMemory();

    bool Update(msg_interface::msg::KeyFramePose &current_laser_odom, Eigen::MatrixXd &pose_covariance);
    void addLoopFactor(const msg_interface::msg::LoopPose &loop_pose);
    void addGNSSFactor(const nav_msgs::msg::Odometry &gnss_pose);
    void addLaserOdomFactor(const msg_interface::msg::KeyFramePose &laser_odom);
    bool keyFrameEmpty();
    bool correctPoses();
    void updatePath(const PointTypePose &pose_in);
    bool globalPathUpdated();
    void getGlobalPath(nav_msgs::msg::Path &global_path);
    void getKeyFrames(sensor_msgs::msg::PointCloud2 &key_frames);

private:
    // gtsam
    std::shared_ptr<gtsam::ISAM2> isam2_ptr_;
    gtsam::NonlinearFactorGraph gtsam_graph_;
    gtsam::Values initial_estimate_;
    gtsam::Values optimized_estimate_;
    gtsam::Values isam_current_estimate_;
    gtsam::ISAM2Params isam_params_;

    nav_msgs::msg::Path global_path_;
    std::string map_frame_;
    std::string traj_dir_;
    std::string key_frames_dir_;

    bool loop_closed_;
    bool gps_fusion_;
    bool has_key_frame_;
    bool global_path_updated_;

    // params
    float key_frame_distance_threshold_;
    float key_frame_angle_threshold_;
    // key frame buff
    pcl::PointCloud<PointTypePose>::Ptr key_frame_deque_;
    // pose output
    std::ofstream back_end_pose_ofs_;
    std::ofstream gps_pose_ofs_;
    std::ofstream global_opt_pose_ofs_;
    // noise 
    std::vector<float> laser_noise_;
    std::vector<float> gnss_noise_;
};
}
#endif