#ifndef LOCALIZATION_MAPPING_MAP_MATCHING_HPP_
#define LOCALIZATION_MAPPING_MAP_MATCHING_HPP_

#include <vector>
#include <map>
#include <mutex>
#include <omp.h>
#include <cstring>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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
#include "localization_mapping/utils/model/filter/box_filter.hpp"
#include "localization_mapping/utils/sensor_data/costom_point_type.hpp"
#include "localization_mapping/utils/tools/tools.hpp"
#include "localization_mapping/utils/model/registration/ndt_registration.hpp"
#include "msg_interface/msg/cloud_info.hpp"
#include "msg_interface/msg/key_frame_pose.hpp"
#include "msg_interface/msg/key_frame.hpp"
#include "msg_interface/srv/save_map.hpp"


namespace truck_slam{

class MapMatching{

public:
    MapMatching();
    void paramsLoad();
    void allocateMemory();
    void updateInitialGuess();
    bool isInited(){ return is_inited_;}
    void setInitPose(float* init_pose){ for(int i = 0; i < 6; i++) init_gnss_pose_[i] = init_pose[i]; }
    void downsampleCurrentScan(msg_interface::msg::CloudInfo &cloud_info);
    void buildLocalMap();
    void ResetLocalMap(const float &x, const float &y, const float &z);
    bool scan2MapOptimization();
    void extractSurroundingKeyFrames();
    void setCloudInfo(const msg_interface::msg::CloudInfo cloud_info);    
    void setKeyFrame(const msg_interface::msg::KeyFrame& opt_key_frame);
    void extractNearby();
    void extractCloud(const pcl::PointCloud<PointType>::Ptr& cloud_key_frames_3d);
    void cornerOptimization();
    bool mapMatch();
    bool ndtRegistration();
    void surfOptimization();
    void combineOptimizationCoeffs();
    void transformUpdate();
    bool LMOptimization();
    void scanPointToMap(const PointType& pointIn, PointType& pointOut);
    float constraintTransformation(float value, float limit);
    void getFeatureFromMap(sensor_msgs::msg::PointCloud2 &corner_cloud, sensor_msgs::msg::PointCloud2 & surface_cloud);
    void getGlobalMap(sensor_msgs::msg::PointCloud2 &corner_cloud, sensor_msgs::msg::PointCloud2 & surface_cloud);
    pcl::PointCloud<PointType> getCurrentCornerDS();
    pcl::PointCloud<PointType> getCurrentSurfDS();
    msg_interface::msg::KeyFramePose getTransToMappedPose();
    void setTransToMap(const msg_interface::msg::KeyFramePose &opt_key_pose);
    void saveGlobalMap(const msg_interface::srv::SaveMap::Request::SharedPtr req, pcl::PointCloud<PointType>::Ptr &corner_cloud, pcl::PointCloud<PointType>::Ptr &surf_cloud);
    Eigen::Vector3f getPosition();
    Eigen::Vector3f getEulerAngles();

private:
    // parameters
    bool is_inited_;
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
    int edge_feature_min_valid_num_;
    int surf_feature_min_valid_num_;
    int current_corner_cloud_ds_num_;
    int current_surface_cloud_ds_num_;
    double current_cloud_info_time_;
    std::string matching_method_;
    std::string map_dir_;

    float init_gnss_pose_[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};        // 初始化pose
    float transform_to_mapped_[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   // scan 到 map坐标系下的转换roll pitch yaw x y z

    std_msgs::msg::Header current_cloud_info_stamp_;
    msg_interface::msg::CloudInfo current_cloud_info_;
    
    pcl::PointCloud<PointType>::Ptr corner_global_map_;
    pcl::PointCloud<PointType>::Ptr surf_global_map_;
    pcl::PointCloud<PointType>::Ptr corner_local_map_;
    pcl::PointCloud<PointType>::Ptr corner_local_map_ds_;
    pcl::PointCloud<PointType>::Ptr surf_local_map_ds_;
    pcl::PointCloud<PointType>::Ptr surf_local_map_;

    Eigen::Affine3f scan_to_map_transform_;
    std::vector<PointType> cloud_ori_corner_vec_; // corner point holder for parallel computation
    std::vector<PointType> coeff_sel_corner_vec_;
    std::vector<bool> cloud_ori_corner_flag_;
    std::vector<PointType> cloud_ori_surf_vec_; // surf point holder for parallel computation
    std::vector<PointType> coeff_sel_surf_vec_;
    std::vector<bool> cloud_ori_surf_flag_;

    pcl::PointCloud<PointType>::Ptr cloud_ori_;
    pcl::PointCloud<PointType>::Ptr coeff_sel_;
    pcl::PointCloud<PointType>::Ptr laser_cloud_scan_;
    pcl::PointCloud<PointType>::Ptr current_corner_cloud_;
    pcl::PointCloud<PointType>::Ptr current_surface_cloud_;
    pcl::PointCloud<PointType>::Ptr current_corner_cloud_ds_;
    pcl::PointCloud<PointType>::Ptr current_surface_cloud_ds_;

    pcl::KdTreeFLANN<PointType>::Ptr kd_tree_surrounding_key_frames_;
    pcl::KdTreeFLANN<PointType>::Ptr kd_tree_corner_from_map_;
    pcl::KdTreeFLANN<PointType>::Ptr kd_tree_surface_from_map_;

    pcl::VoxelGrid<PointType> down_size_filter_corner_;
    pcl::VoxelGrid<PointType> down_size_filter_surface_;
    pcl::VoxelGrid<PointType> down_size_filter_icp_;
    pcl::VoxelGrid<PointType> down_size_filter_surrounding_key_frames_;
    std::shared_ptr<BoxFilter<PointType>> corner_local_map_filter_;
    std::shared_ptr<BoxFilter<PointType>> surf_local_map_filter_;

    std::shared_ptr<NDTRegistration> ndt_registration_ptr_;
};
}
#endif