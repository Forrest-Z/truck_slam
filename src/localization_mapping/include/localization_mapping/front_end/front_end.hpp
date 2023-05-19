/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-13 09:33:15
 */
#ifndef LOCALIZATION_MAPPING_FRONT_END_FRONT_END_HPP_
#define LOCALIZATION_MAPPING_FRONT_END_FRONT_END_HPP_

#include <fstream>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <omp.h>
#include <cstring>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/impl/angles.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include <opencv4/opencv2/opencv.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include "localization_mapping/global_defination/global_defination.h"
#include "localization_mapping/utils/sensor_data/costom_point_type.hpp"
#include "localization_mapping/utils/tools/tools.hpp"
#include "msg_interface/msg/cloud_info.hpp"
#include "msg_interface/msg/key_frame_pose.hpp"
#include "msg_interface/msg/key_frame.hpp"
#include "msg_interface/srv/save_map.hpp"


namespace truck_slam{

class FrontEnd{

public:
    FrontEnd();
    void paramsLoad();
    void allocateMemory();
    void updateInitialGuess();
    void downsampleCurrentScan(msg_interface::msg::CloudInfo &cloud_info);
    bool scan2MapOptimization();
    void extractSurroundingKeyFrames();
    void extractNearby();
    void extractCloud(const pcl::PointCloud<PointType>::Ptr& cloud_key_frames_3d);
    void cornerOptimization();
    void surfOptimization();
    void combineOptimizationCoeffs();
    void transformUpdate();
    bool LMOptimization();
    void scanPointToMap(const PointType& pointIn, PointType& pointOut);
    float constraintTransformation(float value, float limit);
    bool isNewKeyFrame();
    void getFeatureFromMap(sensor_msgs::msg::PointCloud2 &corner_cloud, sensor_msgs::msg::PointCloud2 & surface_cloud);
    void getCurrentCornerDS(pcl::PointCloud<PointType> &corner_cloud);
    void getCurrentSurfDS(pcl::PointCloud<PointType> &surf_cloud);
    void getFramePose(msg_interface::msg::KeyFramePose &laser_pose);
    void getBaseOdom(nav_msgs::msg::Odometry &base_odom);
    void setTransToMap(const PointTypePose &opt_key_pose);
    void setInitPose(const float* init_pose);
    void setCloudInfo(const msg_interface::msg::CloudInfo cloud_info);    
    void setKeyFrame(const sensor_msgs::msg::PointCloud2& opt_key_frames);
    bool isInited(){ return is_inited_;}
    void saveGlobalMap(const msg_interface::srv::SaveMap::Request::SharedPtr req, 
                       pcl::PointCloud<PointType>::Ptr &corner_cloud, 
                       pcl::PointCloud<PointType>::Ptr &surf_cloud,
                       pcl::PointCloud<PointType>::Ptr &raw_global_cloud);
    void saveKeyFrame();
private:
    std::string save_map_directory_;
    std::string key_frames_directory_;
    std::string traj_directory_;
    double mapping_process_interval_;       // 建图频率控制
    float init_gnss_pose_[6];               // 初始化pose

    float transform_to_mapped_[6];          // scan 到 map坐标系下的转换roll pitch yaw x y z
    Eigen::Affine3f scan_to_map_transform_;

    std::vector<PointType> cloud_ori_corner_vec_; // corner point holder for parallel computation
    std::vector<PointType> coeff_sel_corner_vec_;
    std::vector<bool> cloud_ori_corner_flag_;
    std::vector<PointType> cloud_ori_surf_vec_; // surf point holder for parallel computation
    std::vector<PointType> coeff_sel_surf_vec_;
    std::vector<bool> cloud_ori_surf_flag_;

    std::unordered_map<int, std::pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> cloud_map_container_;
    std::ofstream opt_pose_ofs_;
    // parameters
    int N_SCAN_;
    int Horizon_SCAN_;
    int number_of_cores_;
    float imu_RPY_weight_;
    float z_tollerance_;
    float rotation_tollerance_;
    float corner_cloud_leaf_size_;
    float surface_cloud_leaf_size_;
    float surrounding_key_frame_density_;
    double surrounding_key_frame_search_radius_;
    float key_frame_distance_threshold_;
    float key_frame_angle_threshold_;
    int edge_feature_min_valid_num_;
    int surf_feature_min_valid_num_;
    std::vector<float> base_to_lidar_vec_;

    int current_corner_cloud_ds_num_;
    int current_surface_cloud_ds_num_;
    double current_cloud_info_time_;
    std_msgs::msg::Header current_cloud_info_stamp_;
    msg_interface::msg::CloudInfo current_cloud_info_;
    pcl::PointCloud<PointType>::Ptr laser_cloud_scan_;
    pcl::PointCloud<PointType>::Ptr cloud_key_frames_3d_;
    pcl::PointCloud<PointTypePose>::Ptr cloud_key_frames_6d_;
    pcl::PointCloud<PointType>::Ptr current_corner_cloud_;
    pcl::PointCloud<PointType>::Ptr current_surface_cloud_;
    pcl::PointCloud<PointType>::Ptr current_corner_cloud_ds_;
    pcl::PointCloud<PointType>::Ptr current_surface_cloud_ds_;
    pcl::PointCloud<PointType>::Ptr current_corner_cloud_ds_map_;
    pcl::PointCloud<PointType>::Ptr current_surface_cloud_ds_map_;

    pcl::PointCloud<PointType>::Ptr corner_cloud_from_map_;
    pcl::PointCloud<PointType>::Ptr corner_cloud_from_map_ds_;
    pcl::PointCloud<PointType>::Ptr surface_cloud_from_map_;
    pcl::PointCloud<PointType>::Ptr surface_cloud_from_map_ds_;

    pcl::KdTreeFLANN<PointType>::Ptr kd_tree_surrounding_key_frames_;
    pcl::KdTreeFLANN<PointType>::Ptr kd_tree_corner_from_map_;
    pcl::KdTreeFLANN<PointType>::Ptr kd_tree_surface_from_map_;

    pcl::VoxelGrid<PointType> down_size_filter_corner_;
    pcl::VoxelGrid<PointType> down_size_filter_surface_;
    pcl::VoxelGrid<PointType> down_size_filter_icp_;
    pcl::VoxelGrid<PointType> down_size_filter_surrounding_key_frames_;

    pcl::PointCloud<PointType>::Ptr cloud_ori_;
    pcl::PointCloud<PointType>::Ptr coeff_sel_;

    cv::Mat matP_;
    bool is_degenerate_;

    std::mutex mtx_;
    bool is_inited_;
};
}
#endif