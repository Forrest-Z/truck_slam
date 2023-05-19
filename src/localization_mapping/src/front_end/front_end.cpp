/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-13 09:32:49
 */
#include "localization_mapping/front_end/front_end.hpp"
#include "glog/logging.h"

namespace truck_slam{

FrontEnd::FrontEnd(){
    paramsLoad();
    allocateMemory();
}

void FrontEnd::paramsLoad(){
    YAML::Node config_node = YAML::LoadFile(WORK_SPACE_PATH + "/config/config.yaml");
    // cpu settings
    number_of_cores_ = config_node["cpu"]["numberOfCores"].as<int>();
    mapping_process_interval_ = config_node["cpu"]["mappingProcessInterval"].as<double>();
    // voxel filter
    corner_cloud_leaf_size_ = config_node["voxel_filter"]["mappingCornerLeafSize"].as<float>();
    surface_cloud_leaf_size_ = config_node["voxel_filter"]["mappingSurfLeafSize"].as<float>();
    // keyframe
    surrounding_key_frame_density_ = config_node["key_frame"]["surroundingKeyframeDensity"].as<float>();
    surrounding_key_frame_search_radius_ = config_node["key_frame"]["surroundingKeyframeSearchRadius"].as<double>();
    key_frame_angle_threshold_ = config_node["key_frame"]["surroundingkeyframeAddingAngleThreshold"].as<float>();
    key_frame_distance_threshold_ = config_node["key_frame"]["surroundingkeyframeAddingDistThreshold"].as<float>();
    // loam settings
    edge_feature_min_valid_num_ = config_node["loam_settings"]["edgeFeatureMinValidNum"].as<int>();
    surf_feature_min_valid_num_ = config_node["loam_settings"]["surfFeatureMinValidNum"].as<int>();
    // lidar
    N_SCAN_ = config_node["lidar"]["N_SCAN"].as<int>();
    Horizon_SCAN_ = config_node["lidar"]["Horizon_SCAN"].as<int>();
    // imu 
    imu_RPY_weight_ = config_node["imu"]["imuRPYWeight"].as<float>();
    // robot motion_constraint 
    z_tollerance_ = config_node["motion_constraint"]["z_tollerance"].as<float>();
    rotation_tollerance_ = config_node["motion_constraint"]["rotation_tollerance"].as<float>();
    // dir
    save_map_directory_ = config_node["dir"]["save_map_directory"].as<std::string>();
    key_frames_directory_ = config_node["dir"]["key_frames_dir"].as<std::string>();
    traj_directory_ = config_node["dir"]["traj_dir"].as<std::string>();
    // extrinsics
    base_to_lidar_vec_ = config_node["extrinsics"]["baselink_to_lidar"].as<std::vector<float>>();
}

void FrontEnd::allocateMemory(){
    opt_pose_ofs_.open(WORK_SPACE_PATH + traj_directory_ + "/opt_pose.txt" , std::ios::trunc);

    down_size_filter_corner_.setLeafSize(corner_cloud_leaf_size_, corner_cloud_leaf_size_, corner_cloud_leaf_size_);
    down_size_filter_surface_.setLeafSize(surface_cloud_leaf_size_, surface_cloud_leaf_size_, surface_cloud_leaf_size_);
    down_size_filter_icp_.setLeafSize(surface_cloud_leaf_size_, surface_cloud_leaf_size_,  surface_cloud_leaf_size_);
    down_size_filter_surrounding_key_frames_.setLeafSize(surrounding_key_frame_density_, surrounding_key_frame_density_, surrounding_key_frame_density_);
    
    current_corner_cloud_.reset(new pcl::PointCloud<PointType>());
    current_surface_cloud_.reset(new pcl::PointCloud<PointType>());
    current_corner_cloud_ds_.reset(new pcl::PointCloud<PointType>());
    current_surface_cloud_ds_.reset(new pcl::PointCloud<PointType>());
    cloud_key_frames_3d_.reset(new pcl::PointCloud<PointType>());
    cloud_key_frames_6d_.reset(new pcl::PointCloud<PointTypePose>());
    corner_cloud_from_map_.reset(new pcl::PointCloud<PointType>());
    surface_cloud_from_map_.reset(new pcl::PointCloud<PointType>());
    corner_cloud_from_map_ds_.reset(new pcl::PointCloud<PointType>());
    surface_cloud_from_map_ds_.reset(new pcl::PointCloud<PointType>());
    current_corner_cloud_ds_map_.reset(new pcl::PointCloud<PointType>());
    current_surface_cloud_ds_map_.reset(new pcl::PointCloud<PointType>());

    laser_cloud_scan_.reset(new pcl::PointCloud<PointType>());
    coeff_sel_.reset(new pcl::PointCloud<PointType>());

    kd_tree_surrounding_key_frames_.reset(new pcl::KdTreeFLANN<PointType>());
    kd_tree_corner_from_map_.reset(new pcl::KdTreeFLANN<PointType>());
    kd_tree_surface_from_map_.reset(new pcl::KdTreeFLANN<PointType>());

    cloud_ori_corner_vec_.resize(N_SCAN_ * Horizon_SCAN_);
    coeff_sel_corner_vec_.resize(N_SCAN_ * Horizon_SCAN_);
    cloud_ori_corner_flag_.resize(N_SCAN_ * Horizon_SCAN_);
    cloud_ori_surf_vec_.resize(N_SCAN_ * Horizon_SCAN_);
    coeff_sel_surf_vec_.resize(N_SCAN_ * Horizon_SCAN_);
    cloud_ori_surf_flag_.resize(N_SCAN_ * Horizon_SCAN_);

    is_degenerate_ = false;
    is_inited_ = false;
    matP_ = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

    for(int i = 0; i < 6; i++)
        transform_to_mapped_[i] = 0.0;
}

void FrontEnd::setCloudInfo(const msg_interface::msg::CloudInfo cloud_info){
    current_cloud_info_ = cloud_info;
    current_cloud_info_stamp_ = current_cloud_info_.header;
    current_cloud_info_time_ = rclcpp::Time(current_cloud_info_.header.stamp).seconds();
    pcl::fromROSMsg(current_cloud_info_.cloud_corner , *current_corner_cloud_);
    pcl::fromROSMsg(current_cloud_info_.cloud_surface , *current_surface_cloud_);
}

/**
 * 当前帧位姿初始化
 * 1、如果是第一帧，用原始imu数据的RPY初始化当前帧位姿（旋转部分）
 * 2、后续帧，用imu里程计计算两帧之间的增量位姿变换，作用于前一帧的激光位姿，得到当前帧激光位姿
*/
void FrontEnd::updateInitialGuess(){
    static Eigen::Affine3f last_imu_transform;
    if (!is_inited_){
        std::memcpy(transform_to_mapped_, init_gnss_pose_, sizeof(init_gnss_pose_));
        // debug
        LOG(INFO) << "system init pose = " << transform_to_mapped_[0] << " , " << transform_to_mapped_[1] << " , " 
        << transform_to_mapped_[2] << " , " << transform_to_mapped_[3] << " , " 
        << transform_to_mapped_[4] << " , " << transform_to_mapped_[5] << std::endl;
        // 当前帧laser系相对于map系的初始pose, 这里为第一帧所以平移设置为0, 并不是gnss的绝对位置
        last_imu_transform = pcl::getTransformation(transform_to_mapped_[3] , transform_to_mapped_[4] , transform_to_mapped_[5],
                                                    transform_to_mapped_[0] , transform_to_mapped_[1] , transform_to_mapped_[2]);        
        // tranform to base link
        for(int i = 0; i < 6; i++)
            transform_to_mapped_[i] += base_to_lidar_vec_[i];
        is_inited_ = true;
        return;
    }

    //直接使用根据上一帧pose增量初始化当前帧pose  
    Eigen::Affine3f trans_back = pcl::getTransformation(transform_to_mapped_[3] , transform_to_mapped_[4] , transform_to_mapped_[5],
                                        transform_to_mapped_[0] , transform_to_mapped_[1] , transform_to_mapped_[2]);
    Eigen::Affine3f trans_incre = last_imu_transform.inverse() * trans_back;
    Eigen::Affine3f trans_front = pcl::getTransformation(transform_to_mapped_[3], transform_to_mapped_[4], transform_to_mapped_[5],
                                                         transform_to_mapped_[0], transform_to_mapped_[1], transform_to_mapped_[2]);
    Eigen::Affine3f trans_new_back = trans_front * trans_incre;
    pcl::getTranslationAndEulerAngles(trans_new_back, transform_to_mapped_[3], transform_to_mapped_[4], transform_to_mapped_[5],
                                      transform_to_mapped_[0], transform_to_mapped_[1], transform_to_mapped_[2]);
    last_imu_transform = trans_front;

    // debug
    LOG(INFO) << "init pose = " << transform_to_mapped_[0] << " , " << transform_to_mapped_[1] << " , " 
    << transform_to_mapped_[2] << " , " << transform_to_mapped_[3] << " , " 
    << transform_to_mapped_[4] << " , " << transform_to_mapped_[5] << std::endl;
}

/**
 * 提取局部角点、平面点云集合，加入局部map
 * 1、对最近的一帧关键帧，搜索时空维度上相邻的关键帧集合，降采样一下
 * 2、对关键帧集合中的每一帧，提取对应的角点、平面点，加入局部map中
*/
void FrontEnd::extractSurroundingKeyFrames(){
    // 如果还没有关键帧,则不进行匹配
    if(cloud_key_frames_3d_->points.empty()){
        LOG(INFO) << "cloud_key_poses_3d is empty ... \n";
        return;
    }
    extractNearby();
}

/**
 * 对最近的一帧关键帧，搜索时空维度上相邻的关键帧集合并降采样
*/
void FrontEnd::extractNearby(){
    pcl::PointCloud<PointType>::Ptr surrounding_key_frames(new pcl::PointCloud<PointType>());       // 周围的关键帧
    pcl::PointCloud<PointType>::Ptr surrounding_key_frames_ds(new pcl::PointCloud<PointType>());
    std::vector<int> point_search_ind;
    std::vector<float> point_search_sq_dis;
    kd_tree_surrounding_key_frames_->setInputCloud(cloud_key_frames_3d_);
    kd_tree_surrounding_key_frames_->radiusSearch(cloud_key_frames_3d_->back(), surrounding_key_frame_search_radius_, point_search_ind, point_search_sq_dis);
    
    // 得到周围radius内的关键帧点
    for(int i = 0; i < (int)point_search_ind.size(); i++){
        int index = point_search_ind[i];
        surrounding_key_frames->push_back(cloud_key_frames_3d_->points[index]);
    }

    // 降采样
    down_size_filter_surrounding_key_frames_.setInputCloud(surrounding_key_frames);
    down_size_filter_surrounding_key_frames_.filter(*surrounding_key_frames_ds);

    // 得到降采样后的每个点(代表一个关键帧)在cloud_key_frames_3d中的最近关键帧,并将其索引存入intensity
    // 这里的intensity是关键帧的索引，只不过这里借用intensity结构来存放
    for(auto& p : surrounding_key_frames_ds->points){
        kd_tree_surrounding_key_frames_->nearestKSearch(p , 1 , point_search_ind, point_search_sq_dis);
        p.intensity = cloud_key_frames_3d_->points[point_search_ind[0]].intensity;
    }

    //提取了一些时间上最新的关键帧，以防机器人在一个位置原地旋转, 车辆一般不会出现这种情况
    int points_num = cloud_key_frames_3d_->points.size();
    for(int i = points_num-1; i >= 0; i--){
        if(current_cloud_info_time_ - cloud_key_frames_6d_->points[i].time < 10.0)  // 10s 内
            surrounding_key_frames_ds->push_back(cloud_key_frames_3d_->points[i]);
        else 
            continue;
    }

    //按照空间和时间寻找到了用于匹配的关键帧索引, 构建localmap
    // [注]: surrounding_key_frames_ds存放的是关键帧的3d pose,其中intensity存放的是每一帧最近的关键帧索引
    extractCloud(surrounding_key_frames_ds);
}

/**
 * 对关键帧集合中的每一帧，提取对应的角点、平面点，加入局部map中
*/
void FrontEnd::extractCloud(const pcl::PointCloud<PointType>::Ptr& surrounding_key_frames){
    // corner_cloud_from_map_ 和 surface_cloud_from_map_分别是角点和平面点的localmap
    // LOG(INFO) << "surrounding_key_frames.size() = " << surrounding_key_frames->size() << std::endl;
    // LOG(INFO) << "corner_cloud_key_frames_.size() = " << corner_cloud_key_frames_.size() << std::endl;
    // LOG(INFO) << "surface_cloud_key_frames_.size() = " << surface_cloud_key_frames_.size() << std::endl;
    corner_cloud_from_map_->clear();
    surface_cloud_from_map_->clear();
    LOG(INFO) << "surrounding_key_frames size : " << surrounding_key_frames->size() << std::endl;
    for(int i = 0; i < (int)surrounding_key_frames->size(); i++){
        // 如果和最新的关键帧距离过大,则忽略
        if(pointDistance(surrounding_key_frames->points[i], cloud_key_frames_3d_->back()) > surrounding_key_frame_search_radius_)
            continue;
        // 根据索引直接读取点云, 将关键帧的特征点根据对应的pose 转为map系下
        int index = (int)surrounding_key_frames->points[i].intensity;
        if(cloud_map_container_.find(index) != cloud_map_container_.end()){
            *corner_cloud_from_map_ += *transformPointCloud(cloud_map_container_[index].first, &cloud_key_frames_6d_->points[index]);
            *surface_cloud_from_map_ += *transformPointCloud(cloud_map_container_[index].second, &cloud_key_frames_6d_->points[index]);
        }else {
            std::string cloud_path = WORK_SPACE_PATH + key_frames_directory_;
            std::string corner_cloud_name = cloud_path + "/corner/key_frame_" + std::to_string(index) + ".pcd";
            std::string surface_cloud_name = cloud_path + "/surface/key_frame_" + std::to_string(index) + ".pcd";
            pcl::PointCloud<PointType> cloud_corner, cloud_surface;
            pcl::io::loadPCDFile<PointType>(corner_cloud_name, cloud_corner);
            pcl::io::loadPCDFile<PointType>(surface_cloud_name, cloud_surface);
            *corner_cloud_from_map_  += *transformPointCloud(cloud_corner, &cloud_key_frames_6d_->points[index]);
            *surface_cloud_from_map_ += *transformPointCloud(cloud_surface, &cloud_key_frames_6d_->points[index]);
            cloud_map_container_[index] = std::make_pair(cloud_corner, cloud_surface);
        }
    }
    // 下采样
    down_size_filter_corner_.setInputCloud(corner_cloud_from_map_);
    down_size_filter_corner_.filter(*corner_cloud_from_map_ds_);
    down_size_filter_surface_.setInputCloud(surface_cloud_from_map_);
    down_size_filter_surface_.filter(*surface_cloud_from_map_ds_);

    // 清缓存
    if(cloud_map_container_.size() > 1000)
        cloud_map_container_.clear();
}

void FrontEnd::getFeatureFromMap(sensor_msgs::msg::PointCloud2 &corner_cloud, sensor_msgs::msg::PointCloud2 & surface_cloud){
    pcl::toROSMsg(*corner_cloud_from_map_ds_, corner_cloud);
    pcl::toROSMsg(*surface_cloud_from_map_ds_, surface_cloud); 
}


// 当前帧进行降采样
void FrontEnd::downsampleCurrentScan(msg_interface::msg::CloudInfo &cloud_info){
    current_corner_cloud_ds_->clear();
    down_size_filter_corner_.setInputCloud(current_corner_cloud_);
    down_size_filter_corner_.filter(*current_corner_cloud_ds_);
    pcl::toROSMsg(*current_corner_cloud_ds_, cloud_info.cloud_corner);
    current_corner_cloud_ds_num_ = current_corner_cloud_ds_->size();
    
    current_surface_cloud_ds_->clear();
    down_size_filter_surface_.setInputCloud(current_surface_cloud_);
    down_size_filter_surface_.filter(*current_surface_cloud_ds_);
    pcl::toROSMsg(*current_surface_cloud_ds_, cloud_info.cloud_surface);
    current_surface_cloud_ds_num_ = current_surface_cloud_ds_->size();
}

/* scan to map 非线性优化
根据现有地图与最新点云数据进行配准从而更新机器人精确位姿与融合建图，
它分为角点优化、平面点优化、配准与更新等部分。
优化的过程与里程计的计算类似，是通过计算点到直线或平面的距离，构建优化公式再用LM法求解。 */
// ? [idea1]: 港大Mars实验室发布了一种增量kd树(https://github.com/hku-mars/ikd-Tree)可以加速搜索
// ? [idea2]: faster_lio使用了增量式体素结构iVox(https://github.com/gaoxiang12/faster-lio)，增和查的效率比ikd-Tree更优
bool FrontEnd::scan2MapOptimization(){
    if(cloud_key_frames_3d_->points.empty())
        return false;
    
    // 降采样之后判断特征数量是否高于阈值
    if(current_corner_cloud_ds_num_ > edge_feature_min_valid_num_ && current_surface_cloud_ds_num_ > surf_feature_min_valid_num_){
        // 设置kdtree 
        // LOG(INFO) << "corner_cloud_from_map_ds_ size : " << corner_cloud_from_map_ds_->size() << std::endl;
        // LOG(INFO) << "surface_cloud_from_map_ds_ size : " << surface_cloud_from_map_ds_->size() << std::endl;
        kd_tree_corner_from_map_->setInputCloud(corner_cloud_from_map_ds_);
        kd_tree_surface_from_map_->setInputCloud(surface_cloud_from_map_ds_);
        // 优化迭代次数
        LOG(INFO) << "start to optimization ... \n";
        for(int iter = 0; iter < 30; iter++){
            laser_cloud_scan_->clear();
            coeff_sel_->clear();
            // 角点优化
            // auto t1 = std::chrono::system_clock::now(); 
            cornerOptimization();
            // auto t2 = std::chrono::system_clock::now(); 
            // LOG(INFO) << "cornerOptimization cost time : " << std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count() << "us. \n";
            // 平面点优化
            surfOptimization();
            // auto t3 = std::chrono::system_clock::now(); 
            // LOG(INFO) << "surfOptimization cost time : " << std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count() << "us. \n";
            // 联合优化
            combineOptimizationCoeffs();
            // auto t4 = std::chrono::system_clock::now(); 
            // LOG(INFO) << "combineOptimizationCoeffs cost time : " << std::chrono::duration_cast<std::chrono::microseconds>(t4-t3).count() << "us. \n";
            // 判断是否收敛
            if(LMOptimization()) break;            
        }
        // 用imu原始RPY数据与scan-to-map优化后的位姿进行加权融合，更新当前帧位姿的roll、pitch，约束z坐标
        transformUpdate();
        
        // debug
        LOG(INFO) << "LMOptimization pose = " << transform_to_mapped_[0] << " , " << transform_to_mapped_[1] << " , " 
        << transform_to_mapped_[2] << " , " << transform_to_mapped_[3] << " , " 
        << transform_to_mapped_[4] << " , " << transform_to_mapped_[5] << std::endl;
    }else {
        LOG(ERROR) << "Not enough features! Only " << current_corner_cloud_ds_num_ <<" edge and " 
                   << current_surface_cloud_ds_num_ << " planar features available.\n";
        return false;
    }

    return true;
}

// 将点转化为map系
void FrontEnd::scanPointToMap(const PointType& pointIn, PointType& pointOut){
    pointOut.x = scan_to_map_transform_(0,0) * pointIn.x + scan_to_map_transform_(0,1) * pointIn.y + scan_to_map_transform_(0,2) * pointIn.z + scan_to_map_transform_(0,3);
    pointOut.y = scan_to_map_transform_(1,0) * pointIn.x + scan_to_map_transform_(1,1) * pointIn.y + scan_to_map_transform_(1,2) * pointIn.z + scan_to_map_transform_(1,3);   
    pointOut.z = scan_to_map_transform_(2,0) * pointIn.x + scan_to_map_transform_(2,1) * pointIn.y + scan_to_map_transform_(2,2) * pointIn.z + scan_to_map_transform_(2,3);   
    pointOut.intensity = pointIn.intensity;
}

/**
 * 当前激光帧角点寻找局部map匹配点
 * 1、更新当前帧位姿，将当前帧角点坐标变换到map系下，在局部map中查找5个最近点，距离小于1m，且5个点构成直线（用距离中心点的协方差矩阵，特征值进行判断），则认为匹配上了
 * 2、计算当前帧角点到直线的距离、垂线的单位向量，存储为角点参数
*/
void FrontEnd::cornerOptimization(){
    // 得到scan to map的变换
    scan_to_map_transform_ = pcl::getTransformation(transform_to_mapped_[3], transform_to_mapped_[4], transform_to_mapped_[5],
                                                    transform_to_mapped_[0], transform_to_mapped_[1], transform_to_mapped_[2]);
    
    current_corner_cloud_ds_map_.reset(new pcl::PointCloud<PointType>());
    // 将当前帧转为map坐标系下 openMP加速,也可以用别的方法, 例如指令集SSE, tbb, cuda
    #pragma omp parallel for num_threads(number_of_cores_)
    for(int i = 0; i < current_corner_cloud_ds_num_; i++){
        PointType point_scan, point_map, coeff;
        std::vector<int> point_search_index;
        std::vector<float> point_search_dis;
        point_scan = current_corner_cloud_ds_->points[i];
        scanPointToMap(point_scan, point_map);      // 转为map系下的点
        kd_tree_corner_from_map_->nearestKSearch(point_map, 5, point_search_index, point_search_dis);

        cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));                                // 5个点构成的协方差
        cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));                                // 三个特征值
        cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));                                // 三个向量

        // 最大的那个点的距离不能超过1m, 求协方差矩阵
        if(point_search_dis[4] < 1.0){
            float cx = 0, cy = 0, cz = 0;
            // 求出5个点的质心
            for(int j = 0; j < 5; j++){
                cx += corner_cloud_from_map_ds_->points[point_search_index[j]].x;
                cy += corner_cloud_from_map_ds_->points[point_search_index[j]].y;
                cz += corner_cloud_from_map_ds_->points[point_search_index[j]].z;
            }
            cx /= 5; cy /= 5; cz /= 5;

            // 求出协方差
            float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
            for(int j = 0; j < 5; j++){
                float ax = corner_cloud_from_map_ds_->points[point_search_index[j]].x - cx;
                float ay = corner_cloud_from_map_ds_->points[point_search_index[j]].y - cy;
                float az = corner_cloud_from_map_ds_->points[point_search_index[j]].z - cz;

                a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                a22 += ay * ay; a23 += ay * az;
                a33 += az * az;
            }
            a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;  
            matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
            matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
            matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

            // 协方差矩阵特征值分解
            cv::eigen(matA1, matD1, matV1);

            // 如果最大的特征值相比次大特征值，大很多，认为构成了线，角点是合格的
            if(matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)){
                // 局部map对应中心角点，沿着特征向量（直线方向）方向，前后各取一个点
                float x0 = point_map.x;
                float y0 = point_map.y;
                float z0 = point_map.z;

                float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                float z2 = cz - 0.1 * matV1.at<float>(0, 2);
            
                // 求出x0到直线x1x2的距离
                float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                            + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                            - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                            + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                float ld2 = a012 / l12;

                // 下面涉及到一个鲁棒核函数，作者简单地设计了这个核函数。
                // 距离越大，s越小，是个距离惩罚因子（权重）
                float s = 1 - 0.9 * fabs(ld2);
                coeff.x = s * la;
                coeff.y = s * lb;
                coeff.z = s * lc;
                // intensity本质上构成了一个核函数，ld2越接近于1，增长越慢
                // intensity=(1-0.9*distance)*distance=distance-0.9*distance*distance
                coeff.intensity = s * ld2;
                // s>0.1 也就是要求点到直线的距离ld2要小于1m
                // s越大说明ld2越小(离边缘线越近)，这样就说明点pointOri在直线上
                if(s > 0.1){
                    cloud_ori_corner_vec_[i] = point_scan;
                    coeff_sel_corner_vec_[i] = coeff;
                    cloud_ori_corner_flag_[i] = true;
                }
            }
        }
    }
}

/**
 * 当前激光帧平面点寻找局部map匹配点
 * 1、更新当前帧位姿，将当前帧平面点坐标变换到map系下，在局部map中查找5个最近点，距离小于1m，且5个点构成平面（最小二乘拟合平面），则认为匹配上了
 * 2、计算当前帧平面点到平面的距离、垂线的单位向量，存储为平面点参数
*/
void FrontEnd::surfOptimization(){
    scan_to_map_transform_ = pcl::getTransformation(transform_to_mapped_[3], transform_to_mapped_[4], transform_to_mapped_[5],
                                                    transform_to_mapped_[0], transform_to_mapped_[1], transform_to_mapped_[2]);
    current_surface_cloud_ds_map_.reset(new pcl::PointCloud<PointType>());
    #pragma omp parallel for num_threads(number_of_cores_)
    for(int i = 0; i < current_surface_cloud_ds_num_; i++){
        PointType point_scan, point_map, coeff;
        std::vector<int> point_search_index;
        std::vector<float> point_search_dis;
        point_scan = current_surface_cloud_ds_->points[i];
        scanPointToMap(point_scan , point_map);

        // auto t2 = std::chrono::system_clock::now(); 
        kd_tree_surface_from_map_->nearestKSearch(point_map, 5, point_search_index, point_search_dis);
        // auto t3 = std::chrono::system_clock::now(); 
        // LOG(INFO) << "nearestKSearch cost time : " << std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << "ns. \n";
        
        //求解AX=B的X,也就是AX+BY+CZ+1=0
        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();                   

        if(point_search_dis[4] < 1.0){
            for(int j = 0; j < 5; j++){
                matA0(j , 0) = surface_cloud_from_map_ds_->points[point_search_index[j]].x;
                matA0(j , 1) = surface_cloud_from_map_ds_->points[point_search_index[j]].y;
                matA0(j , 2) = surface_cloud_from_map_ds_->points[point_search_index[j]].z;
            }

            // auto t4 = std::chrono::system_clock::now();
            matX0 = matA0.colPivHouseholderQr().solve(matB0);                       
            // auto t5 = std::chrono::system_clock::now(); 
            // LOG(INFO) << "solve cost time : " << std::chrono::duration_cast<std::chrono::nanoseconds>(t5-t4).count() << "ns. \n";

            // 假设平面方程为ax+by+cz+1=0，这里就是求方程的系数abc，d=1
            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;
            float norm = std::sqrt(pa*pa + pb*pb + pc*pc);
            pa /= norm; pb /= norm; pc /= norm; pd /= norm; // 归一化

            // 检查平面是否合格，如果5个点中有点到平面的距离超过0.2m，那么认为这些点太分散了，不构成平面
            bool plane_is_valid = true;
            for(int j = 0; j < 5; j++){
                if(std::fabs(pa * surface_cloud_from_map_ds_->points[point_search_index[j]].x + 
                             pb * surface_cloud_from_map_ds_->points[point_search_index[j]].y +
                             pc * surface_cloud_from_map_ds_->points[point_search_index[j]].z + pd) > 0.2){
                    plane_is_valid = false;
                    break;
                }
            }

            if(plane_is_valid){
                // 当前点到平面距离
                float pd2 = pa * point_map.x + pb * point_map.y + pc * point_map.z + pd;
                // 距离越大，s越小，是个距离惩罚因子（权重）
                // 后面部分相除求的是[pa,pb,pc,pd]与point_map的夹角余弦值(两个sqrt，其实并不是余弦值)
                // 这个夹角余弦值越小越好，越小证明所求的[pa,pb,pc,pd]与平面越垂直
                float s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(std::sqrt(point_scan.x * point_scan.x
                        + point_scan.y * point_scan.y + point_scan.z * point_scan.z));

                // 点到平面垂线单位法向量（其实等价于平面法向量）
                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                coeff.intensity = s * pd2;
                // LOG(INFO) << "[surface_coeff:" << i << "] " << coeff.x << " , " << coeff.y << " , " << coeff.z << " , " << coeff.intensity << std::endl;

                if(s > 0.1){
                    cloud_ori_surf_vec_[i] = point_scan;
                    coeff_sel_surf_vec_[i] = coeff;
                    cloud_ori_surf_flag_[i] = true;
                }
            }
        }
    }
}

/**
 * 提取当前帧中与局部map匹配上了的角点、平面点，加入同一集合
*/
void FrontEnd::combineOptimizationCoeffs(){
    for(int i = 0; i < current_corner_cloud_ds_num_; i++){
        if(cloud_ori_corner_flag_[i]){
            laser_cloud_scan_->push_back(cloud_ori_corner_vec_[i]);
            coeff_sel_->push_back(coeff_sel_corner_vec_[i]);
        }
    }

    for(int i = 0; i < current_surface_cloud_ds_num_; i++){
        if(cloud_ori_surf_flag_[i]){
            laser_cloud_scan_->push_back(cloud_ori_surf_vec_[i]);
            coeff_sel_->push_back(coeff_sel_surf_vec_[i]);
        }
    }

    std::fill(cloud_ori_corner_flag_.begin(), cloud_ori_corner_flag_.end(), false);
    std::fill(cloud_ori_surf_flag_.begin(), cloud_ori_surf_flag_.end(), false);
}

/**
 * scan-to-map优化
 * 对匹配特征点计算Jacobian矩阵，观测值为特征点到直线、平面的距离，构建高斯牛顿方程，迭代优化当前位姿，存transformTobeMapped    
*/
bool FrontEnd::LMOptimization(){
    // 构建点线距离对姿态的雅可比
    // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
    // lidar <- camera      ---     camera <- lidar
    // x = z                ---     x = y
    // y = x                ---     y = z
    // z = y                ---     z = x
    // roll = yaw           ---     roll = pitch
    // pitch = roll         ---     pitch = yaw
    // yaw = pitch          ---     yaw = roll
    // auto t1 = std::chrono::system_clock::now(); 
    // lidar -> camera
    float srx = std::sin(transform_to_mapped_[1]);
    float crx = std::cos(transform_to_mapped_[1]);
    float sry = std::sin(transform_to_mapped_[2]);
    float cry = std::cos(transform_to_mapped_[2]);
    float srz = std::sin(transform_to_mapped_[0]);
    float crz = std::cos(transform_to_mapped_[0]);
    // 判断特征点数量
    const int cloud_sel_num = laser_cloud_scan_->size();
    if(cloud_sel_num < 50){
        LOG(INFO) << "The number of points is too small ... \n";
        return false;
    }

    cv::Mat matA(cloud_sel_num, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, cloud_sel_num, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(cloud_sel_num, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

    PointType point_scan, coeff;
    for(int i = 0; i < cloud_sel_num; i++){
        // lidar->camera
        point_scan.x = laser_cloud_scan_->points[i].y;
        point_scan.y = laser_cloud_scan_->points[i].z;
        point_scan.z = laser_cloud_scan_->points[i].x;

        // lidar->camera
        coeff.x = coeff_sel_->points[i].y;
        coeff.y = coeff_sel_->points[i].z;
        coeff.z = coeff_sel_->points[i].x;
        coeff.intensity = coeff_sel_->points[i].intensity;

        // in camera
        // 说明见： https://blog.csdn.net/weixin_37835423/article/details/111587379
        float arx = (crx*sry*srz*point_scan.x + crx*crz*sry*point_scan.y - srx*sry*point_scan.z) * coeff.x
                  + (-srx*srz*point_scan.x - crz*srx*point_scan.y - crx*point_scan.z) * coeff.y
                  + (crx*cry*srz*point_scan.x + crx*cry*crz*point_scan.y - cry*srx*point_scan.z) * coeff.z;

        float ary = ((cry*srx*srz - crz*sry)*point_scan.x 
                  + (sry*srz + cry*crz*srx)*point_scan.y + crx*cry*point_scan.z) * coeff.x
                  + ((-cry*crz - srx*sry*srz)*point_scan.x 
                  + (cry*srz - crz*srx*sry)*point_scan.y - crx*sry*point_scan.z) * coeff.z;

        float arz = ((crz*srx*sry - cry*srz)*point_scan.x + (-cry*crz-srx*sry*srz)*point_scan.y)*coeff.x
                  + (crx*crz*point_scan.x - crx*srz*point_scan.y) * coeff.y
                  + ((sry*srz + cry*crz*srx)*point_scan.x + (crz*sry-cry*srx*srz)*point_scan.y)*coeff.z;

        // camera -> lidar
        matA.at<float>(i, 0) = arz;
        matA.at<float>(i, 1) = arx;
        matA.at<float>(i, 2) = ary;
        matA.at<float>(i, 3) = coeff.z;
        matA.at<float>(i, 4) = coeff.x;
        matA.at<float>(i, 5) = coeff.y;
        matB.at<float>(i, 0) = -coeff.intensity;
    }
    cv::transpose(matA, matAt);
    matAtA = matAt * matA;  //  H = J^TJ
    matAtB = matAt * matB;  //  B = -J^TX
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);     // 求解 HX=B的线性方程，得到优化增量matX

    transform_to_mapped_[0] += matX.at<float>(0,0);
    transform_to_mapped_[1] += matX.at<float>(1,0);
    transform_to_mapped_[2] += matX.at<float>(2,0);
    transform_to_mapped_[3] += matX.at<float>(3,0);
    transform_to_mapped_[4] += matX.at<float>(4,0);
    transform_to_mapped_[5] += matX.at<float>(5,0);

    float delta_R = std::sqrt(
                        std::pow(pcl::rad2deg(matX.at<float>(0,0)), 2) +
                        std::pow(pcl::rad2deg(matX.at<float>(1,0)), 2) +
                        std::pow(pcl::rad2deg(matX.at<float>(2,0)), 2));

    float delta_t = std::sqrt(
                        std::pow(matX.at<float>(3,0) * 100, 2) +
                        std::pow(matX.at<float>(4,0) * 100, 2) +
                        std::pow(matX.at<float>(5,0) * 100, 2));
    // 收敛判断
    if(delta_R < 0.05 && delta_t < 0.05)
        return true;
        
    return false;
}

/**
 * 用imu原始RPY数据与scan-to-map优化后的位姿进行加权融合，更新当前帧位姿的roll、pitch，约束z坐标
*/
void FrontEnd::transformUpdate(){
    if(current_cloud_info_.imu_available){
        if(std::fabs(current_cloud_info_.imu_pitch_init) < 1.4){

            double imu_weight = imu_RPY_weight_;
            tf2::Quaternion imu_quaternion;
            tf2::Quaternion transform_quaternion;
            double roll_mid, pitch_mid, yaw_mid;

            // roll角求加权均值，用scan-to-map优化得到的位姿与imu原始RPY数据，进行加权平均
            transform_quaternion.setRPY(transform_to_mapped_[0] , 0 , 0);
            imu_quaternion.setRPY(current_cloud_info_.imu_roll_init, 0 , 0);
            tf2::Matrix3x3(transform_quaternion.slerp(imu_quaternion, imu_weight)).getRPY(roll_mid , pitch_mid, yaw_mid);
            transform_to_mapped_[0] = roll_mid;

            // slerp pitch
            // pitch角求加权均值，用scan-to-map优化得到的位姿与imu原始RPY数据，进行加权平均
            transform_quaternion.setRPY(0, transform_to_mapped_[1], 0);
            imu_quaternion.setRPY(0, current_cloud_info_.imu_pitch_init, 0);
            tf2::Matrix3x3(transform_quaternion.slerp(imu_quaternion, imu_weight)).getRPY(roll_mid , pitch_mid, yaw_mid);
            transform_to_mapped_[1] = pitch_mid;
        }
    }

    // 更新当前帧位姿的roll, pitch, z坐标；因为是车，roll、pitch是相对稳定的，
    // 不会有很大变动，一定程度上可以信赖imu的数据，z是进行高度约束
    transform_to_mapped_[0] = constraintTransformation(transform_to_mapped_[0] , rotation_tollerance_);
    transform_to_mapped_[1] = constraintTransformation(transform_to_mapped_[1] , rotation_tollerance_);
    transform_to_mapped_[5] = constraintTransformation(transform_to_mapped_[5] , z_tollerance_);
}

float FrontEnd::constraintTransformation(float value, float limit){
    if (value < -limit)
        value = -limit;
    if (value > limit)
        value = limit;

    return value;
}

bool FrontEnd::isNewKeyFrame(){
    // 如果是第一帧则默认关键帧
    if(cloud_key_frames_3d_->points.empty())
        return true;
    PointTypePose last_key_frame = cloud_key_frames_6d_->back();
    Eigen::Affine3f trans_start = pcl::getTransformation(last_key_frame.x, last_key_frame.y,
                                                         last_key_frame.z, last_key_frame.roll,
                                                         last_key_frame.pitch, last_key_frame.yaw);
    Eigen::Affine3f trans_final = pcl::getTransformation(transform_to_mapped_[3], transform_to_mapped_[4], transform_to_mapped_[5],
                                                         transform_to_mapped_[0], transform_to_mapped_[1], transform_to_mapped_[2]);
    Eigen::Affine3f trans_between = trans_start.inverse() * trans_final;
    float x, y, z, roll, pitch, yaw;
    // 根据角度、距离差判断是否关键帧
    pcl::getTranslationAndEulerAngles(trans_between, x, y, z, roll, pitch, yaw);
    if(std::fabs(roll) < key_frame_angle_threshold_ && std::fabs(pitch) < key_frame_angle_threshold_ && 
       std::fabs(yaw) < key_frame_angle_threshold_ && std::sqrt(x*x + y*y + z*z) < key_frame_distance_threshold_)
       return false;
    return true;
}

void FrontEnd::setKeyFrame(const sensor_msgs::msg::PointCloud2& opt_key_frames){
    // update keyframe pose
    cloud_key_frames_3d_->clear();
    cloud_key_frames_6d_->clear();
    pcl::fromROSMsg(opt_key_frames, *cloud_key_frames_6d_);
    //update tranform to map
    setTransToMap(cloud_key_frames_6d_->points.back());
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

void FrontEnd::getFramePose(msg_interface::msg::KeyFramePose &laser_pose){
    laser_pose.index = cloud_key_frames_3d_->size();
    laser_pose.is_key_frame = isNewKeyFrame();
    laser_pose.time = current_cloud_info_time_;
    laser_pose.x = transform_to_mapped_[3];
    laser_pose.y = transform_to_mapped_[4];
    laser_pose.z = transform_to_mapped_[5];
    laser_pose.roll = transform_to_mapped_[0];
    laser_pose.pitch = transform_to_mapped_[1];
    laser_pose.yaw = transform_to_mapped_[2];
}

void FrontEnd::getBaseOdom(nav_msgs::msg::Odometry &base_odom){
    base_odom.pose.pose.position.x = transform_to_mapped_[3] - base_to_lidar_vec_[3];
    base_odom.pose.pose.position.y = transform_to_mapped_[4] - base_to_lidar_vec_[4];
    base_odom.pose.pose.position.z = transform_to_mapped_[5] - base_to_lidar_vec_[5];

    tf2::Quaternion q; 
    q.setRPY(transform_to_mapped_[0] - base_to_lidar_vec_[0],
             transform_to_mapped_[1] - base_to_lidar_vec_[1],
             transform_to_mapped_[2] - base_to_lidar_vec_[2]);
    base_odom.pose.pose.orientation.w = q.w();
    base_odom.pose.pose.orientation.x = q.x();
    base_odom.pose.pose.orientation.y = q.y();
    base_odom.pose.pose.orientation.z = q.z();
}

void FrontEnd::getCurrentCornerDS(pcl::PointCloud<PointType> &corner_cloud){
    corner_cloud = *current_corner_cloud_ds_;
}

void FrontEnd::getCurrentSurfDS(pcl::PointCloud<PointType> &surf_cloud){
    surf_cloud = *current_surface_cloud_ds_;
}

void FrontEnd::setTransToMap(const PointTypePose &opt_key_pose){
    transform_to_mapped_[0] = opt_key_pose.roll;
    transform_to_mapped_[1] = opt_key_pose.pitch;
    transform_to_mapped_[2] = opt_key_pose.yaw;
    transform_to_mapped_[3] = opt_key_pose.x;
    transform_to_mapped_[4] = opt_key_pose.y;
    transform_to_mapped_[5] = opt_key_pose.z;
}

void FrontEnd::setInitPose(const float* init_pose){ 
    size_t size = sizeof(init_pose); 
    std::memcpy(init_gnss_pose_, init_pose, size); 
}

void FrontEnd::saveGlobalMap(const msg_interface::srv::SaveMap::Request::SharedPtr req, 
                             pcl::PointCloud<PointType>::Ptr &corner_cloud, 
                             pcl::PointCloud<PointType>::Ptr &surf_cloud,
                             pcl::PointCloud<PointType>::Ptr &raw_global_cloud){

    LOG(INFO) << "****************************************************" << std::endl;
    LOG(INFO) << "Saving map to pcd files ..." << std::endl;
    if(req->destination.empty()) 
        save_map_directory_ = WORK_SPACE_PATH + save_map_directory_;
    else 
        save_map_directory_ = std::getenv("HOME") + req->destination;
    LOG(INFO) << "Save destination: " << save_map_directory_ << std::endl;

    // create directory and remove old files;
    int unused = system((std::string("exec rm -r ") + save_map_directory_).c_str());
    unused = system((std::string("mkdir -p ") + save_map_directory_).c_str());

    // save key frame transformations
    pcl::io::savePCDFileBinary(save_map_directory_ + "/trajectory.pcd", *cloud_key_frames_3d_);
    pcl::io::savePCDFileBinary(save_map_directory_ + "/transformations.pcd", *cloud_key_frames_6d_);

    // extract global point cloud map
    pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapCloudDS(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)cloud_key_frames_3d_->size(); i++) {
        pcl::PointCloud<PointType> corner, surface, raw_cloud;
        std::string corner_cloud_name = WORK_SPACE_PATH + key_frames_directory_ + "/corner/key_frame_" + std::to_string(i) + ".pcd";
        std::string surface_cloud_name = WORK_SPACE_PATH + key_frames_directory_ + "/surface/key_frame_" + std::to_string(i) + ".pcd";
        std::string raw_cloud_name = WORK_SPACE_PATH + key_frames_directory_ + "/raw/key_frame_" + std::to_string(i) + ".pcd";
        pcl::io::loadPCDFile<PointType>(corner_cloud_name, corner);
        pcl::io::loadPCDFile<PointType>(surface_cloud_name, surface);
        pcl::io::loadPCDFile<PointType>(raw_cloud_name, raw_cloud);
        *globalCornerCloud += *transformPointCloud(corner,  &cloud_key_frames_6d_->points[i]);
        *globalSurfCloud   += *transformPointCloud(surface,    &cloud_key_frames_6d_->points[i]);
        *globalMapCloud    += *transformPointCloud(raw_cloud,    &cloud_key_frames_6d_->points[i]);

        // save pose to ofs
        Eigen::Matrix4f pose = pcl::getTransformation(cloud_key_frames_6d_->points[i].x, cloud_key_frames_6d_->points[i].y, cloud_key_frames_6d_->points[i].z,
                                                      cloud_key_frames_6d_->points[i].roll, cloud_key_frames_6d_->points[i].pitch, cloud_key_frames_6d_->points[i].yaw).matrix();
        savePose(opt_pose_ofs_, pose);
        LOG(INFO) << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloud_key_frames_6d_->size() << " ...";
    }

    if(req->resolution != 0)
    {
        LOG(INFO) << "\n\nSave resolution: " << req->resolution << std::endl;
        // down-sample and save corner cloud
        down_size_filter_corner_.setInputCloud(globalCornerCloud);
        down_size_filter_corner_.setLeafSize(req->resolution, req->resolution, req->resolution);
        down_size_filter_corner_.filter(*globalCornerCloudDS);
        pcl::io::savePCDFileBinary(save_map_directory_ + "/CornerMap.pcd", *globalCornerCloudDS);
        // down-sample and save surf cloud
        down_size_filter_surface_.setInputCloud(globalSurfCloud);
        down_size_filter_surface_.setLeafSize(req->resolution, req->resolution, req->resolution);
        down_size_filter_surface_.filter(*globalSurfCloudDS);
        pcl::io::savePCDFileBinary(save_map_directory_ + "/SurfMap.pcd", *globalSurfCloudDS);   
        // down-sample and save raw global map
        down_size_filter_surface_.setInputCloud(globalMapCloud);
        down_size_filter_surface_.setLeafSize(req->resolution, req->resolution, req->resolution);
        down_size_filter_surface_.filter(*globalMapCloudDS);
        pcl::io::savePCDFileBinary(save_map_directory_ + "/RawGlobalMap.pcd", *globalMapCloud);    
        pcl::io::savePCDFileBinary(save_map_directory_ + "/GlobalMapDS.pcd", *globalMapCloudDS);    
        corner_cloud.reset(new pcl::PointCloud<PointType>(*globalCornerCloudDS));
        surf_cloud.reset(new pcl::PointCloud<PointType>(*globalSurfCloudDS));
        raw_global_cloud.reset(new pcl::PointCloud<PointType>(*globalMapCloudDS));
    }
}

void FrontEnd::saveKeyFrame(){
    auto t1 = std::chrono::system_clock::now();
    std::string save_path = WORK_SPACE_PATH + key_frames_directory_;
    int index = cloud_key_frames_3d_->size();
    pcl::PointCloud<PointType>::Ptr corner_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surface_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr raw_cloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(current_cloud_info_.cloud_corner, *corner_cloud);
    pcl::fromROSMsg(current_cloud_info_.cloud_surface, *surface_cloud);
    pcl::fromROSMsg(current_cloud_info_.cloud_raw, *raw_cloud);
    std::string cloud_name = save_path + "/corner/key_frame_" + std::to_string(index) + ".pcd";
    pcl::io::savePCDFileBinary(cloud_name, *corner_cloud);
    cloud_name = save_path + "/surface/key_frame_" + std::to_string(index) + ".pcd";
    pcl::io::savePCDFileBinary(cloud_name, *surface_cloud);
    cloud_name = save_path + "/raw/key_frame_" + std::to_string(index) + ".pcd";
    pcl::io::savePCDFileBinary(cloud_name, *raw_cloud);
    auto t2 = std::chrono::system_clock::now();
    LOG(INFO) << "saveKeyFrame cost time : " << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() << " ms." << std::endl;
}
}
