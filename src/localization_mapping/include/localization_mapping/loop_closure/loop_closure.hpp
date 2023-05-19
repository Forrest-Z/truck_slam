/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-03-06 10:16:37
 */
#ifndef LOCALIZATION_MAPPING_LOOP_CLOSURE_LOOP_CLOSURE_HPP_
#define LOCALIZATION_MAPPING_LOOP_CLOSURE_LOOP_CLOSURE_HPP_

#include <vector>
#include <unordered_map>
#include <mutex>
#include <map>
#include <yaml-cpp/yaml.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include "glog/logging.h"

#include <tf2/LinearMath/Quaternion.h>

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

#include "localization_mapping/utils/sensor_data/costom_point_type.hpp"
#include "localization_mapping/global_defination/global_defination.h"
#include "localization_mapping/utils/tools/tools.hpp"
#include "localization_mapping/utils/model/registration/icp_registration.hpp"
#include "msg_interface/msg/loop_pose.hpp"
#include "msg_interface/msg/key_frame.hpp"



namespace truck_slam{
class LoopClosure{
public:
    using PointType = pcl::PointXYZI;
    using PointTypePose = PointXYZIRPYT;
    
    LoopClosure();
    void paramsLoad();
    void allocateMemory();
    bool Detect(msg_interface::msg::LoopPose &loop_pose, 
                pcl::PointCloud<PointType>::Ptr &cur_key_frame_cloud,
                pcl::PointCloud<PointType>::Ptr &pre_key_frame_cloud);
    bool detectLoopClosure(int &loop_index_cur, int &loop_index_pre);
    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& near_key_frames, const int& key, const int& search_num);
    void setKeyFrames(const sensor_msgs::msg::PointCloud2& key_frames);

private:
    pcl::PointCloud<PointType>::Ptr cloud_key_frames_3d_;
    pcl::PointCloud<PointTypePose>::Ptr cloud_key_frames_6d_;
    std::unordered_map<int, std::pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> cloud_map_container_;
    std::unordered_map<int, int> loop_index_container_;
    pcl::KdTreeFLANN<PointType>::Ptr kd_tree_history_key_poses_;
    pcl::KdTreeFLANN<PointType>::Ptr kd_tree_surrounding_points_;
    pcl::VoxelGrid<PointType> down_size_filter_;
    float filter_leaf_size_;
    int history_search_num_;
    std::string key_frames_directory_;
    float history_key_frame_fitness_score_;
    float history_key_frame_search_radius_;
    float history_key_frame_search_time_diff_;
    std::shared_ptr<ICPRegistration> icp_registration_ptr_;
    std::vector<float> noise_;
};
}
#endif