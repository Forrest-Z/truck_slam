#include "localization_mapping/map_matching/map_matching.hpp"
#include "glog/logging.h"

namespace truck_slam{

MapMatching::MapMatching(){
    paramsLoad();
    allocateMemory();
}

void MapMatching::paramsLoad(){
    YAML::Node tmp_node = YAML::LoadFile(WORK_SPACE_PATH + "/config/map_localization.yaml");
    YAML::Node config_node = tmp_node["map_matching_node"]["ros__parameters"];
    corner_cloud_leaf_size_ = config_node["mappingCornerLeafSize"].as<float>();
    surface_cloud_leaf_size_ = config_node["mappingSurfLeafSize"].as<float>();
    surrounding_key_frame_density_ = config_node["surroundingKeyframeDensity"].as<float>();
    surrounding_key_frame_search_radius_ = config_node["surroundingKeyframeSearchRadius"].as<double>();
    number_of_cores_ = config_node["numberOfCores"].as<int>();
    edge_feature_min_valid_num_ = config_node["edgeFeatureMinValidNum"].as<int>();
    surf_feature_min_valid_num_ = config_node["surfFeatureMinValidNum"].as<int>();
    N_SCAN_ = config_node["N_SCAN"].as<int>();
    Horizon_SCAN_ = config_node["Horizon_SCAN"].as<int>();
    imu_RPY_weight_ = config_node["imuRPYWeight"].as<float>();
    z_tollerance_ = config_node["z_tollerance"].as<float>();
    rotation_tollerance_ = config_node["rotation_tollerance"].as<float>();
    matching_method_ = config_node["matching_method"].as<std::string>();
    map_dir_ = config_node["map_path"].as<std::string>();
    corner_local_map_filter_ = std::make_shared<BoxFilter<PointType>>(config_node);
    surf_local_map_filter_   = std::make_shared<BoxFilter<PointType>>(config_node);
    ndt_registration_ptr_ = std::make_shared<NDTRegistration>(config_node["NDT"]);
}

void MapMatching::allocateMemory(){
    down_size_filter_corner_.setLeafSize(corner_cloud_leaf_size_, corner_cloud_leaf_size_, corner_cloud_leaf_size_);
    down_size_filter_surface_.setLeafSize(surface_cloud_leaf_size_, surface_cloud_leaf_size_, surface_cloud_leaf_size_);
    down_size_filter_icp_.setLeafSize(surface_cloud_leaf_size_, surface_cloud_leaf_size_,  surface_cloud_leaf_size_);
    down_size_filter_surrounding_key_frames_.setLeafSize(surrounding_key_frame_density_, surrounding_key_frame_density_, surrounding_key_frame_density_);

    // load global map
    corner_global_map_.reset(new pcl::PointCloud<PointType>());
    surf_global_map_.reset(new pcl::PointCloud<PointType>());
    pcl::io::loadPCDFile<PointType>(WORK_SPACE_PATH + map_dir_ + "/CornerMap.pcd", *corner_global_map_);
    pcl::io::loadPCDFile<PointType>(WORK_SPACE_PATH + map_dir_ + "/SurfMap.pcd", *surf_global_map_);

    surf_local_map_.reset(new pcl::PointCloud<PointType>());
    corner_local_map_.reset(new pcl::PointCloud<PointType>());
    surf_local_map_ds_.reset(new pcl::PointCloud<PointType>());
    corner_local_map_ds_.reset(new pcl::PointCloud<PointType>());
    current_corner_cloud_.reset(new pcl::PointCloud<PointType>());
    current_surface_cloud_.reset(new pcl::PointCloud<PointType>());
    current_corner_cloud_ds_.reset(new pcl::PointCloud<PointType>());
    current_surface_cloud_ds_.reset(new pcl::PointCloud<PointType>());

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
    is_inited_ = false;
}

void MapMatching::setCloudInfo(const msg_interface::msg::CloudInfo cloud_info){
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
void MapMatching::updateInitialGuess(){
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
        is_inited_ = true; 
        ResetLocalMap(transform_to_mapped_[3], transform_to_mapped_[4], transform_to_mapped_[5]);
        return;
    }
    // debug
    LOG(INFO) << "init laser pose = " << transform_to_mapped_[0] << " , " << transform_to_mapped_[1] << " , " 
              << transform_to_mapped_[2] << " , " << transform_to_mapped_[3] << " , " 
              << transform_to_mapped_[4] << " , " << transform_to_mapped_[5] << std::endl;

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

// 构建局部地图
void  MapMatching::buildLocalMap(){
    // 匹配之后判断是否需要更新局部地图
    std::vector<float> edge = corner_local_map_filter_->GetEdge();
    for (int i = 0; i < 3; i++) {
        if (fabs(transform_to_mapped_[i+3] - edge.at(2 * i)) > 20.0 && fabs(transform_to_mapped_[i+3] - edge.at(2 * i + 1)) > 20.0) {
            continue;
        }
            
        ResetLocalMap(transform_to_mapped_[3], transform_to_mapped_[4], transform_to_mapped_[5]);
        break;
    }
}

void MapMatching::ResetLocalMap(const float &x, const float &y, const float &z){
    std::vector<float> origin = {x, y, z};
    // use ROI filtering for local map segmentation:
    corner_local_map_filter_->SetOrigin(origin);
    surf_local_map_filter_->SetOrigin(origin);
    corner_local_map_filter_->Filter(corner_global_map_, corner_local_map_);
    surf_local_map_filter_->Filter(surf_global_map_, surf_local_map_);

    // downsize local map
    down_size_filter_corner_.setInputCloud(corner_local_map_);
    down_size_filter_corner_.filter(*corner_local_map_ds_);
    down_size_filter_surface_.setInputCloud(surf_local_map_);
    down_size_filter_surface_.filter(*surf_local_map_ds_);

    std::vector<float> edge = corner_local_map_filter_->GetEdge();
    LOG(INFO) << "New local map:" << edge.at(0) << ","
                                  << edge.at(1) << ","
                                  << edge.at(2) << ","
                                  << edge.at(3) << ","
                                  << edge.at(4) << ","
                                  << edge.at(5) << std::endl << std::endl;
}


// 当前帧进行降采样
void MapMatching::downsampleCurrentScan(msg_interface::msg::CloudInfo &cloud_info){
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

// scan to map 
bool MapMatching::mapMatch(){
    if(matching_method_ == "LOAM")
        return scan2MapOptimization();
    else if(matching_method_ == "NDT")
        return ndtRegistration();

    LOG(INFO) << "Error, Matching method must be 'LOAM' or 'NDT' ! \n";
    return false;
}

// ndt 配准
bool MapMatching::ndtRegistration(){
    Eigen::Matrix4f predict_pose = pcl::getTransformation(transform_to_mapped_[3], transform_to_mapped_[4], transform_to_mapped_[5],
                                                              transform_to_mapped_[0], transform_to_mapped_[1], transform_to_mapped_[2]).matrix();
    // ndt registration 
    Eigen::Matrix4f corner_cloud_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f surf_cloud_pose = Eigen::Matrix4f::Identity();
    pcl::PointCloud<PointType>::Ptr result_cloud_ptr(new pcl::PointCloud<PointType>());
    ndt_registration_ptr_->SetInputTarget(surf_local_map_ds_);
    ndt_registration_ptr_->ScanMatch(current_surface_cloud_ds_, predict_pose, result_cloud_ptr, surf_cloud_pose);
    // ndt_registration_ptr_->SetInputTarget(corner_local_map_ds_);
    // ndt_registration_ptr_->ScanMatch(current_corner_cloud_ds_, surf_cloud_pose, result_cloud_ptr, corner_cloud_pose);
    // transform 
    transToArray(surf_cloud_pose, transform_to_mapped_);
    return true;
}


/* scan to map 非线性优化
根据现有地图与最新点云数据进行配准从而更新机器人精确位姿与融合建图，
它分为角点优化、平面点优化、配准与更新等部分。
优化的过程与里程计的计算类似，是通过计算点到直线或平面的距离，构建优化公式再用LM法求解。 */
// ? [idea1]: 港大Mars实验室发布了一种增量kd树(https://github.com/hku-mars/ikd-Tree)可以加速搜索
// ? [idea2]: faster_lio使用了增量式体素结构iVox(https://github.com/gaoxiang12/faster-lio)，增和查的效率比ikd-Tree更优
bool MapMatching::scan2MapOptimization(){
    // 降采样之后判断特征数量是否高于阈值
    if(current_corner_cloud_ds_num_ > edge_feature_min_valid_num_ && current_surface_cloud_ds_num_ > surf_feature_min_valid_num_){
        // 设置kdtree 
        kd_tree_corner_from_map_->setInputCloud(corner_local_map_ds_);
        kd_tree_surface_from_map_->setInputCloud(surf_local_map_ds_);
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
void MapMatching::scanPointToMap(const PointType& pointIn, PointType& pointOut){
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
void MapMatching::cornerOptimization(){
    // 得到scan to map的变换
    scan_to_map_transform_ = pcl::getTransformation(transform_to_mapped_[3], transform_to_mapped_[4], transform_to_mapped_[5],
                                                    transform_to_mapped_[0], transform_to_mapped_[1], transform_to_mapped_[2]);
    
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
                cx += corner_local_map_->points[point_search_index[j]].x;
                cy += corner_local_map_->points[point_search_index[j]].y;
                cz += corner_local_map_->points[point_search_index[j]].z;
            }
            cx /= 5; cy /= 5; cz /= 5;

            // 求出协方差
            float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
            for(int j = 0; j < 5; j++){
                float ax = corner_local_map_->points[point_search_index[j]].x - cx;
                float ay = corner_local_map_->points[point_search_index[j]].y - cy;
                float az = corner_local_map_->points[point_search_index[j]].z - cz;

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
void MapMatching::surfOptimization(){
    scan_to_map_transform_ = pcl::getTransformation(transform_to_mapped_[3], transform_to_mapped_[4], transform_to_mapped_[5],
                                                    transform_to_mapped_[0], transform_to_mapped_[1], transform_to_mapped_[2]);
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
                matA0(j , 0) = surf_local_map_->points[point_search_index[j]].x;
                matA0(j , 1) = surf_local_map_->points[point_search_index[j]].y;
                matA0(j , 2) = surf_local_map_->points[point_search_index[j]].z;
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
                if(std::fabs(pa * surf_local_map_->points[point_search_index[j]].x + 
                             pb * surf_local_map_->points[point_search_index[j]].y +
                             pc * surf_local_map_->points[point_search_index[j]].z + pd) > 0.2){
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
void MapMatching::combineOptimizationCoeffs(){
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
bool MapMatching::LMOptimization(){
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
void MapMatching::transformUpdate(){
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

float MapMatching::constraintTransformation(float value, float limit){
    if (value < -limit)
        value = -limit;
    if (value > limit)
        value = limit;

    return value;
}

Eigen::Vector3f MapMatching::getPosition(){
    return Eigen::Vector3f(transform_to_mapped_[3], transform_to_mapped_[4], transform_to_mapped_[5]);
}

// roll pitch yaw
Eigen::Vector3f MapMatching::getEulerAngles(){
    return Eigen::Vector3f(transform_to_mapped_[0], transform_to_mapped_[1], transform_to_mapped_[2]);
}

void MapMatching::getFeatureFromMap(sensor_msgs::msg::PointCloud2 &corner_cloud, sensor_msgs::msg::PointCloud2 & surface_cloud){
    pcl::toROSMsg(*corner_local_map_ds_, corner_cloud);
    pcl::toROSMsg(*surf_local_map_ds_, surface_cloud); 
}

void MapMatching::getGlobalMap(sensor_msgs::msg::PointCloud2 &corner_cloud, sensor_msgs::msg::PointCloud2 & surface_cloud){
    pcl::toROSMsg(*corner_global_map_, corner_cloud);
    pcl::toROSMsg(*surf_global_map_, surface_cloud); 
}

pcl::PointCloud<PointType> MapMatching::getCurrentCornerDS(){
    return *current_corner_cloud_ds_;
}

pcl::PointCloud<PointType> MapMatching::getCurrentSurfDS(){
    return *current_surface_cloud_ds_;
}
}
