/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-23 11:51:19
 */
#include "localization_mapping/back_end/back_end.hpp"

namespace truck_slam{
BackEnd::BackEnd(){
    paramsLoad();
    allocateMemory();
}
void BackEnd::paramsLoad(){
    YAML::Node config_node = YAML::LoadFile(WORK_SPACE_PATH + "/config/config.yaml");
    map_frame_ = config_node["frame"]["mapFrame"].as<std::string>();
    laser_noise_ = config_node["noise"]["lidar_odometry_noise"].as<std::vector<float>>();
    gnss_noise_ = config_node["noise"]["gnss_position_noise"].as<std::vector<float>>();
    traj_dir_ = config_node["dir"]["traj_dir"].as<std::string>();
    key_frames_dir_ = config_node["dir"]["key_frames_dir"].as<std::string>();
    key_frame_angle_threshold_ = config_node["key_frame"]["surroundingkeyframeAddingAngleThreshold"].as<float>();
    key_frame_distance_threshold_ = config_node["key_frame"]["surroundingkeyframeAddingDistThreshold"].as<float>();
}

void BackEnd::allocateMemory(){
    key_frame_deque_.reset(new pcl::PointCloud<PointTypePose>());
    isam_params_.relinearizeSkip = 1;
    isam_params_.setRelinearizeThreshold(0.1);
    isam2_ptr_ = std::make_shared<gtsam::ISAM2>(isam_params_);
    loop_closed_ = false;
    gps_fusion_ = false;
    has_key_frame_ = false;
    global_path_updated_ = false;
    gps_pose_ofs_.open(WORK_SPACE_PATH + traj_dir_ + "/gps_pose.txt" , std::ios::trunc);
    back_end_pose_ofs_.open(WORK_SPACE_PATH + traj_dir_ + "/back_end_pose.txt" , std::ios::trunc);
}

bool BackEnd::Update(msg_interface::msg::KeyFramePose &current_laser_odom, Eigen::MatrixXd &pose_covariance){
    // update iSAM
    isam2_ptr_->update(gtsam_graph_, initial_estimate_);
    isam2_ptr_->update();

    if(loop_closed_ || gps_fusion_){
        isam2_ptr_->update();
        isam2_ptr_->update();
        isam2_ptr_->update();
        isam2_ptr_->update();
        isam2_ptr_->update();
    }

    gtsam_graph_.resize(0);
    initial_estimate_.clear();

    // save key poses;
    gtsam::Pose3 latest_estimate;
    isam_current_estimate_ = isam2_ptr_->calculateEstimate();           // 得到所有优化后的关键帧位姿
    latest_estimate = isam_current_estimate_.at<gtsam::Pose3>(isam_current_estimate_.size() - 1);   // 取出最新的关键帧pose

    // save laser pose
    current_laser_odom.x = latest_estimate.translation().x();
    current_laser_odom.y = latest_estimate.translation().y();
    current_laser_odom.z = latest_estimate.translation().z();
    current_laser_odom.roll = latest_estimate.rotation().roll();
    current_laser_odom.pitch = latest_estimate.rotation().pitch();
    current_laser_odom.yaw = latest_estimate.rotation().yaw();
    current_laser_odom.index = key_frame_deque_->size();
    PointTypePose opt_laser_odom;
    transToPointPose(current_laser_odom, opt_laser_odom);
    key_frame_deque_->points.push_back(opt_laser_odom);
    pose_covariance = isam2_ptr_->marginalCovariance(isam_current_estimate_.size() - 1);
    // 加入当前关键帧pose 到global path
    updatePath(opt_laser_odom);
    // save pose to ofs
    Eigen::Matrix4f pose = pcl::getTransformation(current_laser_odom.x, current_laser_odom.y, current_laser_odom.z,
                                                  current_laser_odom.roll, current_laser_odom.pitch, current_laser_odom.yaw).matrix();
    savePose(back_end_pose_ofs_, pose);
    // 如果有gnss和回环因子加入优化则 更新所有关键帧的位姿
    correctPoses();
    LOG(INFO) << " key_frame_poses = " <<  key_frame_deque_->size() << std::endl;
    return true;
}

bool BackEnd::correctPoses(){
    if(keyFrameEmpty())
        return false;
    if(loop_closed_ || gps_fusion_){
        LOG(INFO) << "global path update ... \n";
        global_path_.poses.clear();
        int num_pose = isam_current_estimate_.size();
        global_opt_pose_ofs_.open(WORK_SPACE_PATH + traj_dir_ + "/global_opt_pose.txt" , std::ios::trunc);
        for(int i = 0; i < num_pose; i++){
            key_frame_deque_->points[i].x = isam_current_estimate_.at<gtsam::Pose3>(i).translation().x();
            key_frame_deque_->points[i].y = isam_current_estimate_.at<gtsam::Pose3>(i).translation().y();
            key_frame_deque_->points[i].z = isam_current_estimate_.at<gtsam::Pose3>(i).translation().z();
            key_frame_deque_->points[i].roll = isam_current_estimate_.at<gtsam::Pose3>(i).rotation().roll();
            key_frame_deque_->points[i].pitch = isam_current_estimate_.at<gtsam::Pose3>(i).rotation().pitch();
            key_frame_deque_->points[i].yaw = isam_current_estimate_.at<gtsam::Pose3>(i).rotation().yaw();
            updatePath(key_frame_deque_->points[i]);
            // save pose to ofs
            Eigen::Matrix4f pose = pcl::getTransformation(key_frame_deque_->points[i].x, key_frame_deque_->points[i].y, key_frame_deque_->points[i].z,
                                                          key_frame_deque_->points[i].roll, key_frame_deque_->points[i].pitch, key_frame_deque_->points[i].yaw).matrix();
            savePose(global_opt_pose_ofs_, pose);
        }
        loop_closed_ = false;
        gps_fusion_ = false;
        global_opt_pose_ofs_.close();
        return true;
    }
    return false;
}

void BackEnd::addLaserOdomFactor(const msg_interface::msg::KeyFramePose &laser_odom){
    gtsam::Pose3 laser_pose3 = truck_slam::transToPose3(laser_odom);
    gtsam::noiseModel::Diagonal::shared_ptr noise_laser = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << laser_noise_[3], laser_noise_[4], laser_noise_[5], laser_noise_[0], laser_noise_[1], laser_noise_[2]).finished()); // rad*rad, meter*meter
    if(keyFrameEmpty()){
        gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, laser_pose3, noise_laser));
        initial_estimate_.insert(0, laser_pose3); 
    }else{
        gtsam::Pose3 prev_frame_pose = truck_slam::transToPose3(key_frame_deque_->back());
        gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(key_frame_deque_->size()-1, key_frame_deque_->size(),
                                                            prev_frame_pose.between(laser_pose3), noise_laser));
        initial_estimate_.insert(key_frame_deque_->size(), laser_pose3);
    }
}

void BackEnd::addLoopFactor(const msg_interface::msg::LoopPose &loop_pose){
    gtsam::Rot3 rotation(loop_pose.pose.orientation.w,
                         loop_pose.pose.orientation.x,
                         loop_pose.pose.orientation.y,
                         loop_pose.pose.orientation.z);
    gtsam::Point3 translation(loop_pose.pose.position.x,
                              loop_pose.pose.position.y,
                              loop_pose.pose.position.z);
    gtsam::Pose3 pose_between(rotation, translation);
    gtsam::Vector6 vector6(6);
    vector6 << loop_pose.covariance[0], loop_pose.covariance[1], loop_pose.covariance[2],
               loop_pose.covariance[3], loop_pose.covariance[4], loop_pose.covariance[5];
    gtsam::noiseModel::Diagonal::shared_ptr noise_between =  gtsam::noiseModel::Diagonal::Variances(vector6);

    // index0 : current_key_frame , index1 : prev_key_frame, pose_between: from current_key_frame to prev
    gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(loop_pose.index0, loop_pose.index1, pose_between, noise_between));
    loop_closed_ = true;
}

void BackEnd::addGNSSFactor(const nav_msgs::msg::Odometry &gnss_pose){
    gtsam::Point3 translation(gnss_pose.pose.pose.position.x,
                              gnss_pose.pose.pose.position.y,
                              gnss_pose.pose.pose.position.z);
    gtsam::Vector3 vector3(3);
    vector3 << gnss_noise_[0], gnss_noise_[1], gnss_noise_[2];
    // std::max((float)gnss_pose.pose.covariance[0], 1.0f),
    //            std::max((float)gnss_pose.pose.covariance[7], 1.0f),
    //            std::max((float)gnss_pose.pose.covariance[14], 1.0f);
    gtsam::noiseModel::Diagonal::shared_ptr noise_gnss = gtsam::noiseModel::Diagonal::Variances(vector3);
    gtsam_graph_.add(gtsam::GPSFactor(key_frame_deque_->size(), translation, noise_gnss));

    // save gnss pose
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3,3>(0,0) = Eigen::Quaternionf((float)gnss_pose.pose.pose.orientation.w, (float)gnss_pose.pose.pose.orientation.x, (float)gnss_pose.pose.pose.orientation.y, (float)gnss_pose.pose.pose.orientation.z).toRotationMatrix();
    pose.block<3,1>(0,3) = Eigen::Vector3f(gnss_pose.pose.pose.position.x, gnss_pose.pose.pose.position.y, gnss_pose.pose.pose.position.z);
    savePose(gps_pose_ofs_, pose);

    gps_fusion_ = true;
}

void BackEnd::updatePath(const PointTypePose &pose_in){
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = rclcpp::Time(pose_in.time * 1e9);
    pose_stamped.header.frame_id = map_frame_;
    pose_stamped.pose.position.x = pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z = pose_in.z;
    tf2::Quaternion q; q.setRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
    pose_stamped.pose.orientation.w = q.w();
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    global_path_.poses.push_back(pose_stamped);
}

void BackEnd::getGlobalPath(nav_msgs::msg::Path &global_path){
    global_path.poses.assign(global_path_.poses.begin(), global_path_.poses.end());
}

void BackEnd::getKeyFrames(sensor_msgs::msg::PointCloud2 &key_frames){
    pcl::toROSMsg(*key_frame_deque_, key_frames);
}

bool BackEnd::globalPathUpdated(){
    return global_path_updated_;
}

bool BackEnd::keyFrameEmpty(){
    return key_frame_deque_->empty();
}
}
