/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-03-06 10:44:40
 */
#include "localization_mapping/loop_closure/loop_closure.hpp"

namespace truck_slam{
LoopClosure::LoopClosure(){
    paramsLoad();
    allocateMemory();
}

void LoopClosure::paramsLoad(){
    YAML::Node config_node = YAML::LoadFile(WORK_SPACE_PATH + "/config/config.yaml");
    history_key_frame_search_radius_ = config_node["loop_closure"]["historyKeyframeSearchRadius"].as<float>();
    history_key_frame_search_time_diff_ = config_node["loop_closure"]["historyKeyframeSearchTimeDiff"].as<float>();
    history_search_num_ = config_node["loop_closure"]["historyKeyframeSearchNum"].as<int>();
    history_key_frame_fitness_score_ = config_node["loop_closure"]["historyKeyframeFitnessScore"].as<float>();
    filter_leaf_size_ = config_node["voxel_filter"]["mappingSurfLeafSize"].as<float>();
    noise_ = config_node["noise"]["loop_closure_noise"].as<std::vector<float>>();
    key_frames_directory_ = config_node["dir"]["key_frames_dir"].as<std::string>();
    icp_registration_ptr_ = std::make_shared<ICPRegistration>(config_node["loop_closure"]["icp_settings"]);
}

void LoopClosure::allocateMemory(){
    cloud_key_frames_3d_.reset(new pcl::PointCloud<PointType>());
    cloud_key_frames_6d_.reset(new pcl::PointCloud<PointTypePose>());
    kd_tree_history_key_poses_.reset(new pcl::KdTreeFLANN<PointType>());
    kd_tree_surrounding_points_.reset(new pcl::KdTreeFLANN<PointType>());
    down_size_filter_.setLeafSize(filter_leaf_size_, filter_leaf_size_, filter_leaf_size_);
}

/**
 * 在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧
*/
bool LoopClosure::detectLoopClosure(int &loop_index_cur, int &loop_index_pre){
    int index_cur = cloud_key_frames_3d_->size() - 1;
    int index_prev = -1;
    if(loop_index_container_.find(index_cur) != loop_index_container_.end())
        return false;

    // 找到于当前帧距离最近的历史关键帧
    std::vector<int> point_search_ind;
    std::vector<float> point_search_dis;
    kd_tree_history_key_poses_->setInputCloud(cloud_key_frames_3d_);
    kd_tree_history_key_poses_->radiusSearch(cloud_key_frames_3d_->back(), history_key_frame_search_radius_,
                                             point_search_ind, point_search_dis, 0);
    
    // 挑选出时间间隔超过阈值的一帧作为闭环
    for(size_t i = 0; i < point_search_ind.size(); i++){
        int id = point_search_ind[i];
        if(std::fabs(cloud_key_frames_6d_->points[id].time - cloud_key_frames_6d_->back().time) > history_key_frame_search_time_diff_){
            index_prev = id;
            break;
        }
    }

    if(index_prev == -1 || index_cur == index_prev){
        LOG(INFO) << "index_prev == -1 || index_cur == index_prev ... loop detect failed\n";
        return false;
    }
    loop_index_cur = index_cur;
    loop_index_pre = index_prev;

    return true;
}

void LoopClosure::loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& near_key_frames, const int& key, const int& search_num){
    near_key_frames->clear();
    int cloud_size = cloud_key_frames_6d_->size();
    // 在该帧前后数量为search_num 的范围内进行搜索， 并组成一个小的localmap， 用于回环匹配
    for(int i = -search_num; i <= search_num; i++){
        int key_index = key + i;
        if(key_index < 0 || key_index >= cloud_size)
            continue;
        if(cloud_map_container_.find(key_index) != cloud_map_container_.end()){
            *near_key_frames += *transformPointCloud(cloud_map_container_[key_index].first, &cloud_key_frames_6d_->points[key_index]); 
            *near_key_frames += *transformPointCloud(cloud_map_container_[key_index].second, &cloud_key_frames_6d_->points[key_index]); 
        }else{
            std::string cloud_path = WORK_SPACE_PATH + key_frames_directory_;
            std::string corner_cloud_name = cloud_path + "/corner/key_frame_" + std::to_string(key_index) + ".pcd";
            std::string surface_cloud_name = cloud_path + "/surface/key_frame_" + std::to_string(key_index) + ".pcd";
            pcl::PointCloud<PointType> cloud_corner, cloud_surface;
            pcl::io::loadPCDFile<PointType>(corner_cloud_name, cloud_corner);
            pcl::io::loadPCDFile<PointType>(surface_cloud_name, cloud_surface);
            *near_key_frames  += *transformPointCloud(cloud_corner, &cloud_key_frames_6d_->points[key_index]);
            *near_key_frames  += *transformPointCloud(cloud_surface, &cloud_key_frames_6d_->points[key_index]);
            cloud_map_container_[key_index] = std::make_pair(cloud_corner, cloud_surface);
        }
    }

    if(near_key_frames->empty()){
        LOG(INFO) << "near_key_frames is empty... \n";
        return;
    }

    // 降采样
    down_size_filter_.setInputCloud(near_key_frames);
    down_size_filter_.filter(*near_key_frames);
}

bool LoopClosure::Detect(msg_interface::msg::LoopPose &loop_pose, 
                         pcl::PointCloud<PointType>::Ptr &cur_key_frame_cloud,
                         pcl::PointCloud<PointType>::Ptr &pre_key_frame_cloud){
    if(cloud_key_frames_3d_->points.empty())
        return false;
    
    // 回环检测并找到对应的关键帧id
    int loop_index_cur;
    int loop_index_pre;
    if(!detectLoopClosure(loop_index_cur, loop_index_pre)){
        LOG(INFO) << "can not find loop , retrun ... \n";
        return false; 
    }

    LOG(INFO) << "loop_index_cur : " << loop_index_cur << ",  loop_index_pre: " << loop_index_pre <<  std::endl;


    // 根据找到的关键帧，提取附近的角点和平面点用于匹配
    {   
        loopFindNearKeyframes(cur_key_frame_cloud, loop_index_cur, 0);
        loopFindNearKeyframes(pre_key_frame_cloud, loop_index_pre, history_search_num_);
        // 特征点过少，则不进行匹配
        if(cur_key_frame_cloud->size() < 300 || pre_key_frame_cloud->size() < 1000){
            LOG(INFO) << "cur_key_frame_cloud->size() < 300 || pre_key_frame_cloud->size() < 1000 ... \n";
            return false;
        }
    }
    // icp配准得到回环约束
    pcl::PointCloud<PointType>::Ptr result_cloud_ptr(new pcl::PointCloud<PointType>());
    Eigen::Matrix4f final_transform;
    icp_registration_ptr_->SetInputTarget(pre_key_frame_cloud);
    icp_registration_ptr_->ScanMatch(cur_key_frame_cloud, result_cloud_ptr, final_transform);   // 得到cur到pre的transfrom
    LOG(INFO) << "icp HasConverged : " << icp_registration_ptr_->HasConverged() << " , icp FitnessScore" << icp_registration_ptr_->GetFitnessScore() << std::endl;
    LOG(INFO) << "icp final_transform : " << final_transform << std::endl;
    if(!icp_registration_ptr_->HasConverged() || 
        icp_registration_ptr_->GetFitnessScore() > history_key_frame_fitness_score_)
        return false;

    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correction_laser_frame(final_transform);
    Eigen::Affine3f cur_key_frame_trans = pcl::getTransformation(cloud_key_frames_6d_->points[loop_index_cur].x,
                                                                 cloud_key_frames_6d_->points[loop_index_cur].y,
                                                                 cloud_key_frames_6d_->points[loop_index_cur].z,
                                                                 cloud_key_frames_6d_->points[loop_index_cur].roll,
                                                                 cloud_key_frames_6d_->points[loop_index_cur].pitch,
                                                                 cloud_key_frames_6d_->points[loop_index_cur].yaw);
    
    Eigen::Affine3f pre_key_frame_trans_correction = correction_laser_frame * cur_key_frame_trans;
    pcl::getTranslationAndEulerAngles(pre_key_frame_trans_correction, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 pose_from = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    gtsam::Pose3 pose_to = gtsam::Pose3(gtsam::Rot3::RzRyRx(cloud_key_frames_6d_->points[loop_index_pre].roll,
                                        cloud_key_frames_6d_->points[loop_index_pre].pitch,
                                        cloud_key_frames_6d_->points[loop_index_pre].yaw), 
                                        gtsam::Point3(cloud_key_frames_6d_->points[loop_index_pre].x,
                                        cloud_key_frames_6d_->points[loop_index_pre].y,
                                        cloud_key_frames_6d_->points[loop_index_pre].z));
    
    gtsam::Pose3 between_pose = pose_from.between(pose_to);
    // float noise_score = icp_registration_ptr_->GetFitnessScore();
    tf2::Quaternion q;
    q.setRPY(between_pose.rotation().roll(), between_pose.rotation().pitch(), between_pose.rotation().yaw());
    loop_pose.index0 = loop_index_cur;
    loop_pose.index1 = loop_index_pre;

    loop_pose.pose.position.x = between_pose.translation().x();
    loop_pose.pose.position.y = between_pose.translation().y();
    loop_pose.pose.position.z = between_pose.translation().z();

    loop_pose.pose.orientation.w = q.w();
    loop_pose.pose.orientation.x = q.x();
    loop_pose.pose.orientation.y = q.y();
    loop_pose.pose.orientation.z = q.z();

    loop_pose.covariance.push_back(noise_[3]);
    loop_pose.covariance.push_back(noise_[4]);
    loop_pose.covariance.push_back(noise_[5]);
    loop_pose.covariance.push_back(noise_[0]);
    loop_pose.covariance.push_back(noise_[1]);
    loop_pose.covariance.push_back(noise_[2]);


    loop_index_container_[loop_index_cur] = loop_index_pre;

    return true;
}

void LoopClosure::setKeyFrames(const sensor_msgs::msg::PointCloud2& key_frames){    
    // update keyframe pose
    cloud_key_frames_3d_->clear();
    cloud_key_frames_6d_->clear();
    pcl::fromROSMsg(key_frames, *cloud_key_frames_6d_);
    LOG(INFO) << "key_frame.key_frame_poses.size() : " << cloud_key_frames_6d_->size() << std::endl;
    for(size_t i = 0; i < cloud_key_frames_6d_->size(); i++){
        PointType pose_3d;
        PointTypePose pose_6d = cloud_key_frames_6d_->points[i];
        pose_3d.x = pose_6d.x;
        pose_3d.y = pose_6d.y;
        pose_3d.z = pose_6d.z;
        pose_3d.intensity = pose_6d.intensity;
        cloud_key_frames_3d_->points.push_back(pose_3d);
    }
}
}