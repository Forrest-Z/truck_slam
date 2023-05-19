/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-08 13:41:27
 */

#include "localization_mapping/data_pretreat/data_pretreat.hpp"


namespace truck_slam{

DataPretreat::DataPretreat(){
    loadParameters();
    resetParameters();
}

void DataPretreat::loadParameters(){
    // read params
    YAML::Node config_node = YAML::LoadFile(WORK_SPACE_PATH + "/config/config.yaml");
    // lidar params
    std::string sensorStr = config_node["lidar"]["sensor_type"].as<std::string>();
    N_SCAN_ = config_node["lidar"]["N_SCAN"].as<int>();
    Horizon_SCAN_ = config_node["lidar"]["Horizon_SCAN"].as<int>();
    downsampleRate_ = config_node["lidar"]["downsampleRate"].as<int>();
    lidarMinRange_ = config_node["lidar"]["lidarMinRange"].as<double>();
    lidarMaxRange_ = config_node["lidar"]["lidarMaxRange"].as<double>();
    if(sensorStr == "velodyne")
        sensor_type_ = SensorType::VELODYNE;
    else if(sensorStr == "ouster")
        sensor_type_ = SensorType::OUSTER;
    else if(sensorStr == "rs")
        sensor_type_ = SensorType::Rs;
    else if(sensorStr == "livox")
        sensor_type_ = SensorType::LIVOX;
    else{
        LOG(FATAL) << "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox' or 'Rs'): " << sensorStr << "\n";
        rclcpp::shutdown();
    }    
    
    // down size setting
    odometrySurfLeafSize_ = config_node["voxel_filter"]["odometrySurfLeafSize"].as<float>();
    //imu params
    imu_type_ = config_node["imu"]["imuType"].as<int>();
    // loam settings
    edgeThreshold_ = config_node["loam_settings"]["edgeThreshold"].as<float>();
    surfThreshold_ = config_node["loam_settings"]["surfThreshold"].as<float>();
    // extrinsics
    imu_to_lidar_vec_ = config_node["extrinsics"]["imu_to_lidar"].as<std::vector<float>>();
}

void DataPretreat::resetParameters(){
    // init variables
    imu_index_ = 0;
    for (int i = 0; i < queue_length_; ++i){
        imuTime_[i] = 0; imuRotX_[i] = 0; imuRotY_[i] = 0; imuRotZ_[i] = 0;
    }
    extracted_cloud_.reset(new pcl::PointCloud<PointType>());
    first_point_flag_ = true;
    columnIdnCountVec_.assign(N_SCAN_ , 0);
    start_time_ = end_time_ = 0.0;
    cloud_smoothness_.resize(N_SCAN_ * Horizon_SCAN_);
    cloudCurvature_ = new float[N_SCAN_ * Horizon_SCAN_];
    cloudNeighborPicked_ = new int[N_SCAN_ * Horizon_SCAN_];
    cloudLabel_ = new int[N_SCAN_ * Horizon_SCAN_];
    voxel_filter_.setLeafSize(odometrySurfLeafSize_, odometrySurfLeafSize_, odometrySurfLeafSize_);
    ext_rot_ = Eigen::AngleAxisd(imu_to_lidar_vec_[2], Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(imu_to_lidar_vec_[1], Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(imu_to_lidar_vec_[0], Eigen::Vector3d::UnitX());
    ext_rpy_ = ext_rot_;
    ext_trans_ = Eigen::Vector3d(imu_to_lidar_vec_[3], imu_to_lidar_vec_[4], imu_to_lidar_vec_[5]);
    ext_rpy_q_ = Eigen::Quaterniond(ext_rpy_);
}

void DataPretreat::setStartAndEndTime(const double &start_time , const double &end_time){
    start_time_ = start_time;
    end_time_ = end_time;
}

void DataPretreat::imuDeskewInfo(msg_interface::msg::CloudInfo& cloud_info, std::deque<sensor_msgs::msg::Imu>& imu_deque){
    
    if(end_time_ == 0 || start_time_ == 0){
        LOG(FATAL) << "[imuDeskewInfo]: start or end time of this scan is 0 , please check timestamp is valid ... \n";
        return;
    }
    cloud_info.imu_available = false;
    while(!imu_deque.empty()){
        if(rclcpp::Time(imu_deque.front().header.stamp).seconds() >= (start_time_ - 0.02))
            break;
        
        imu_deque.pop_front();
    }

    if(imu_deque.empty()){
        LOG(INFO) << "imu deque is empty , wait imu data ...\n";
        return;
    }
    // LOG(INFO) << "imu deque size = " << std::to_string(imu_deque.size()) << std::endl;
    // LOG(INFO) << "start time = " <<std::to_string(start_time_) << std::endl;
    // LOG(INFO) << "end time = " << std::to_string(end_time_) << std::endl;

    // LOG(INFO) << "current clou start_time = " << std::to_string(start_time_) << std::endl;
    // LOG(INFO) << "current clou end_time = " << std::to_string(end_time_) << std::endl;
    imu_index_ = 0;
    for(size_t i = 0; i < imu_deque.size(); i++){
        sensor_msgs::msg::Imu imu_data = imu_deque[i];
        double start_imu_time = rclcpp::Time(imu_data.header.stamp).seconds();
        // LOG(INFO) << "start_imu_time = " << std::to_string(start_imu_time) << std::endl;
        // [注]: 这里需要9轴imu输出姿态来初始化
        if(start_imu_time <= start_time_)
            imuRPY2rosRPY(imu_data, cloud_info.imu_roll_init , cloud_info.imu_pitch_init , cloud_info.imu_yaw_init);

        if(start_imu_time > (end_time_ + 0.02))
            break;

        if(imu_index_ == 0){
            imuTime_[imu_index_] = start_imu_time;
            imuRotX_[imu_index_] = 0;
            imuRotY_[imu_index_] = 0;
            imuRotZ_[imu_index_] = 0;
            ++imu_index_;
            continue;
        }

        // get angular velocity
        double angular_x, angular_y, angular_z;
        imuAngular2rosAngular(imu_data, angular_x, angular_y, angular_z);

        // integrate rotation
        // 逐个积分得到每个imu时刻相对于起始时刻的欧拉角, 这里使用的是欧拉积分,如果使用中值积分会不会精度更高?不过会更耗时
        double time_diff = start_imu_time - imuTime_[imu_index_-1];
        imuRotX_[imu_index_] = imuRotX_[imu_index_-1] + angular_x * time_diff;
        imuRotY_[imu_index_] = imuRotY_[imu_index_-1] + angular_y * time_diff;
        imuRotZ_[imu_index_] = imuRotZ_[imu_index_-1] + angular_z * time_diff;
        imuTime_[imu_index_] = start_imu_time;
        ++imu_index_;
    }
    --imu_index_;

    if (imu_index_ <= 0){
        LOG(INFO) << "Get error result : imu_index <= 0 , imu is not available \n";
        return;
    }

    cloud_info.imu_available = true;
}

void DataPretreat::odomDeskewInfo(msg_interface::msg::CloudInfo& cloud_info , std::deque<nav_msgs::msg::Odometry>& odom_deque){
    
    if(end_time_ == 0 || start_time_ == 0){
        LOG(WARNING) << "[odomDeskewInfo]: start or end time of this scan is 0 , please check timestamp is valid ... \n";
        return;
    }
    cloud_info.odom_available = false;
    double this_odom_time = 0.0;
    while (!odom_deque.empty()){
        this_odom_time = rclcpp::Time(odom_deque.front().header.stamp).seconds();
        if(this_odom_time >= (start_time_ - 0.01))
            break;
        
        odom_deque.pop_front();
    }
    if (odom_deque.empty())
        return;
    if (this_odom_time > start_time_){
        LOG(INFO) << "this_odom_time = " << std::to_string(this_odom_time) << " , start_time_ = " << std::to_string(start_time_) << std::endl;
        LOG(INFO) << "Get error result : odom_deque.front().time > start_time \n";
        return;
    }
    // get start odometry at the beinning of the scan
    nav_msgs::msg::Odometry start_odom_msg;
    for (size_t i = 0; i < odom_deque.size(); i++)
    {
        start_odom_msg = odom_deque[i];
        double timestamp = rclcpp::Time(start_odom_msg.header.stamp).seconds();
        if (timestamp < start_time_)
            continue;
        else
            break;
    }
    tf2::Quaternion orientation(start_odom_msg.pose.pose.orientation.x, 
                                start_odom_msg.pose.pose.orientation.y,
                                start_odom_msg.pose.pose.orientation.z,
                                start_odom_msg.pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(orientation).getRPY(roll , pitch , yaw);

    // Initial guess used in mapOptimization
    cloud_info.initial_guess_x = start_odom_msg.pose.pose.position.x;
    cloud_info.initial_guess_y = start_odom_msg.pose.pose.position.y;
    cloud_info.initial_guess_z = start_odom_msg.pose.pose.position.z;
    cloud_info.initial_guess_roll  = roll;
    cloud_info.initial_guess_pitch = pitch;
    cloud_info.initial_guess_yaw   = yaw;
    cloud_info.odom_available = true;

    // get last odometry at the end of the scan
    this_odom_time = rclcpp::Time(odom_deque.back().header.stamp).seconds();
    if(this_odom_time < end_time_){
        LOG(INFO) << "Timestamp of the odometry deque is earlier than the end time of this scan ... \n";
        return;
    }

    nav_msgs::msg::Odometry end_odom_msg;
    for(size_t i = 0; i < odom_deque.size(); i++){
        end_odom_msg = odom_deque[i];
        this_odom_time = rclcpp::Time(end_odom_msg.header.stamp).seconds();
        if(this_odom_time <= end_time_)
            continue;
        else
            break;
    }

    // ?
    if(int(round(start_odom_msg.pose.covariance[0])) != int(round(end_odom_msg.pose.covariance[0]))){
        LOG(WARNING) << "[odomDeskewInfo]: round(start_odom_msg.pose.covariance[0])) != int(round(end_odom_msg.pose.covariance[0]) ... \n";
        return;
    }

    // get transform of the start and end   计算在当前点云帧时间内的pose增量
    Eigen::Affine3f trans_begin = pcl::getTransformation(start_odom_msg.pose.pose.position.x,
                                    start_odom_msg.pose.pose.position.y, start_odom_msg.pose.pose.position.z,
                                    roll, pitch, yaw);
    
    double roll_end , pitch_end , yaw_end;
    tf2::Quaternion quaternion_end(end_odom_msg.pose.pose.orientation.x, 
                                   end_odom_msg.pose.pose.orientation.y,
                                   end_odom_msg.pose.pose.orientation.z,
                                   end_odom_msg.pose.pose.orientation.w);

    tf2::Matrix3x3(quaternion_end).getEulerYPR(yaw_end , pitch_end , roll_end);

    Eigen::Affine3f trans_end = pcl::getTransformation(end_odom_msg.pose.pose.position.x,
                                    end_odom_msg.pose.pose.position.y, end_odom_msg.pose.pose.position.z,
                                    roll_end, pitch_end, yaw_end);
    
    Eigen::Affine3f trans_incre = trans_begin.inverse() * trans_end;
    // 得到当前关键帧起始与结束时odom的增量值
    pcl::getTranslationAndEulerAngles(trans_incre, odomIncreX_, odomIncreY_, odomIncreZ_, rollIncre_, pitchIncre_, yawIncre_);
}
    
void DataPretreat::findRotation(const double& point_time, float *rotXCur, float *rotYCur, float *rotZCur){
    int imu_index_front = 0;
    
    while(imu_index_front < imu_index_){
        if(point_time < imuTime_[imu_index_front])
            break;
        else    
            imu_index_front++;
    }

    if(point_time >= imuTime_[imu_index_front] || imu_index_front == 0){
        *rotXCur = imuRotX_[imu_index_front];
        *rotYCur = imuRotY_[imu_index_front];
        *rotZCur = imuRotZ_[imu_index_front];
    }else{
        int imu_index_back = imu_index_front - 1;
        // 根据点的时间戳作双线性插值，得到点的时间戳对应与点云起始时刻的欧拉角
        double ratioFront = (point_time - imuTime_[imu_index_back]) / (imuTime_[imu_index_front] - imuTime_[imu_index_back]);
        double ratioBack = (imuTime_[imu_index_front] - point_time) / (imuTime_[imu_index_front] - imuTime_[imu_index_back]);
        *rotXCur = imuRotX_[imu_index_front] * ratioFront + imuRotX_[imu_index_back] * ratioBack;
        *rotYCur = imuRotY_[imu_index_front] * ratioFront + imuRotY_[imu_index_back] * ratioBack;
        *rotZCur = imuRotZ_[imu_index_front] * ratioFront + imuRotZ_[imu_index_back] * ratioBack;
    }
}

void DataPretreat::findPosition(const double& rel_time, float *posXCur, float *posYCur, float *posZCur){
    *posXCur = 0; *posYCur = 0; *posZCur = 0;

//     // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

//     // if (cloud_info.odom_available == false || odomDeskewFlag == false)
//     //     return;

//     // float ratio = rel_time / (end_time_ - start_time_);

//     // posXCur = ratio * odomIncreX_;
//     // posYCur = ratio * odomIncreY_;
//     // posZCur = ratio * odomIncreZ_;
}

void DataPretreat::deskewPoint(PointType &point, const double &relTime){    
    // start_time 是点云起始时刻， reltime 是当前点相对于起始点的时间差值
    // double point_time = start_time_ + relTime;      // velodyne
    double point_time = relTime;      // rslidar


    // 根据imu积分的结果找到雷达点时刻的相对于点云起始时刻的旋转欧拉角度
    float rotXCur = 0.0, rotYCur = 0.0, rotZCur = 0.0;
    findRotation(point_time, &rotXCur, &rotYCur, &rotZCur);
    float posXCur = 0.0, posYCur = 0.0, posZCur = 0.0;
    findPosition(relTime, &posXCur, &posYCur, &posZCur);

    if(first_point_flag_){
        // 找到的欧拉角 start point ro cureent point的，所以需要inverse,
        // 说明: 源lio-sam代码中这里直接用pcl的轮子将欧拉角转为Eigen Affine求逆得到, 这里为了加速直接储存起始时刻的欧拉角度
        // trans_start_ = pcl::getTransformation(, posYCur, posZCur, rotXCur, rotYCur, rotZCur).inverse().matrix();
        trans_start_[0] = posXCur; trans_start_[1] = posYCur; trans_start_[2] = posZCur;
        trans_start_[3] = rotXCur; trans_start_[4] = rotYCur; trans_start_[5] = rotZCur;
        first_point_flag_ = false;
    }


    // 将当前点转换到起始时刻位置，完成畸变补偿
    // 注: 这里直接使用欧拉角转旋转矩阵的公式直接求解,并且直接运算得到去畸变的点,为了节约计算时间,
    //     相比于lio-sam源代码,这里避免了pcl::geTransformation()函数的耗时,同时也避免Eigen Affine相乘的耗时,
    //     这里只避免一次矩阵求逆以及矩阵相乘的过程
    {   
        float delta_x = posXCur - trans_start_[0];
        float delta_y = posYCur - trans_start_[1];
        float delta_z = posZCur - trans_start_[2];
        float delta_roll = rotXCur - trans_start_[3];
        float delta_pitch = rotYCur - trans_start_[4];
        float delta_yaw = rotZCur - trans_start_[5];
        float A = std::cos (delta_yaw),  B = sin (delta_yaw),  C  = std::cos (delta_pitch), D  = sin (delta_pitch),
        E = std::cos (delta_roll), F = sin (delta_roll), DE = D*E,         DF = D*F;

        float x = point.x, y = point.y, z = point.z;
        point.x = A*C * x + (A*DF - B*E) * y + (B*F + A*DE) * z + delta_x;
        point.y = B*C * x + (A*E + B*DF) * y + (B*DE - A*F) * z + delta_y;
        point.z = -D *  x +  C*F         * y + C*E          * z + delta_z;
    }
}

void DataPretreat::projectPointCloud(const pcl::PointCloud<PointXYZIRT>::Ptr& current_cloud , cv::Mat& rangeMat, 
                                    pcl::PointCloud<PointType>::Ptr& full_cloud, const bool &imu_available){
    int cloudSize = current_cloud->size();
    for(int i = 0; i < cloudSize; i++){
        PointType thisPoint;
        thisPoint.x = current_cloud->points[i].x;
        thisPoint.y = current_cloud->points[i].y;
        thisPoint.z = current_cloud->points[i].z;
        thisPoint.intensity = current_cloud->points[i].intensity;

        float range = pointRange(thisPoint);
        if(range < lidarMinRange_ || range > lidarMaxRange_)
            continue;

        int rowIndex = current_cloud->points[i].ring; 
        if(rowIndex < 0 || rowIndex >= N_SCAN_)
            continue;

        int colIndex = -1;
        if(sensor_type_ == SensorType::VELODYNE || sensor_type_ == SensorType::Rs ||
           sensor_type_ == SensorType::OUSTER){
            
            float horizonAngle = atan2(thisPoint.x , thisPoint.y) * 180 / M_PI;
            static float ang_res = 360.0 / float(Horizon_SCAN_);
            colIndex =  -round((horizonAngle-90.0)/ang_res) + Horizon_SCAN_/2;
            if(colIndex >= Horizon_SCAN_){
                colIndex -= Horizon_SCAN_;
            }

        }else if(sensor_type_ == SensorType::LIVOX){
            colIndex = columnIdnCountVec_[rowIndex];
            columnIdnCountVec_[rowIndex]++;
        }

        if (colIndex < 0 || colIndex >= Horizon_SCAN_)
            continue;
        
        // LOG(INFO) << "rangeMat push, index: " << rowIndex << " " << colIndex << " " << std::endl;
        if (rangeMat.at<float>(rowIndex, colIndex) != FLT_MAX)
            continue;
        // 运动畸变补偿

        int index = colIndex + rowIndex * Horizon_SCAN_;
        // 这里按照点的顺序采用降采样
        if (index % downsampleRate_ != 0)
            continue;

        if(deskew_flag_ == 1 && imu_available)
            deskewPoint(thisPoint , current_cloud->at(i).timestamp);
        
        rangeMat.at<float>(rowIndex, colIndex) = range;
        full_cloud->at(index) = thisPoint;
    }
}

void DataPretreat::featureExtraction(msg_interface::msg::CloudInfo& cloud_info , pcl::PointCloud<PointType>::Ptr &corner_cloud,
                                     pcl::PointCloud<PointType>::Ptr &surface_cloud){
    // 这里可能会增额外的耗时
    pcl::fromROSMsg(cloud_info.cloud_deskewed , *extracted_cloud_); 

    // 1. calculate smoothness
    calculateSmoothness(cloud_info);


    // 2. mark occluded points
    markOccludedPoints(cloud_info);

    // 3. extract features                     
    extractFeatures(cloud_info, corner_cloud, surface_cloud);


    pcl::toROSMsg(*corner_cloud , cloud_info.cloud_corner);
    pcl::toROSMsg(*surface_cloud , cloud_info.cloud_surface);

    cloud_info.start_ring_index.clear();
    cloud_info.end_ring_index.clear();
    cloud_info.point_col_ind.clear();
    cloud_info.point_range.clear();
}

void DataPretreat::calculateSmoothness(msg_interface::msg::CloudInfo& cloud_info){
    int cloudSize = extracted_cloud_->size();
    // 利用当前点前后5个点,计算曲率,参考loam
    for(int i = 5; i < cloudSize-5; i++){
        float diffRange = cloud_info.point_range[i-5] + cloud_info.point_range[i-4] +
                          cloud_info.point_range[i-3] + cloud_info.point_range[i-2] +
                          cloud_info.point_range[i-1] + cloud_info.point_range[i+1] +
                          cloud_info.point_range[i+2] + cloud_info.point_range[i+3] +
                          cloud_info.point_range[i+4] + cloud_info.point_range[i+5] -
                          cloud_info.point_range[i] * 10;
        cloudCurvature_[i] = diffRange * diffRange;

        cloudNeighborPicked_[i] = 0;    // 0表示还未进行特征提取处理,1表示遮挡、平行，或者已经进行特征提取的点
        cloudLabel_[i] = 0;     // 点标签, 1表示角点，-1表示平面点

        cloud_smoothness_[i].value = cloudCurvature_[i];
        cloud_smoothness_[i].index = i;
    }
}

void DataPretreat::markOccludedPoints(msg_interface::msg::CloudInfo& cloud_info){
    int cloudSize = extracted_cloud_->size();

    for(int i = 5; i < cloudSize-6; i++){
        float depth1 = cloud_info.point_range[i];
        float depth2 = cloud_info.point_range[i+1];

        // 计算相邻的两个点的行索引(1800),目的是判断相邻的两个点是否不再同一个线上
        int colDiff = std::abs(int(cloud_info.point_col_ind[i] - cloud_info.point_col_ind[i+1]));
        if(colDiff < 10){
            // 两个点在同一扫描线上，且距离相差大于0.3，认为存在遮挡关系
            //（也就是这两个点不在同一平面上，如果在同一平面上，距离相差不会太大）
            //  远处的点会被遮挡，标记一下该点以及相邻的5个点，后面不再进行特征提取
            if(depth1 - depth2 > 0.3){
                cloudNeighborPicked_[i] = 1;
                cloudNeighborPicked_[i-1] = 1;
                cloudNeighborPicked_[i-2] = 1;
                cloudNeighborPicked_[i-3] = 1;
                cloudNeighborPicked_[i-4] = 1;
                cloudNeighborPicked_[i-5] = 1;
            }else if(depth2 - depth1 > 0.3){
                cloudNeighborPicked_[i+1] = 1;
                cloudNeighborPicked_[i+2] = 1;
                cloudNeighborPicked_[i+3] = 1;
                cloudNeighborPicked_[i+4] = 1;
                cloudNeighborPicked_[i+5] = 1;    
                cloudNeighborPicked_[i+6] = 1;    
            }
        }

        //如果当前点距离左右邻点都过远，则视其为瑕点，因为入射角可能太小导致误差较大
        float diff1 = std::fabs(cloud_info.point_range[i-1] - cloud_info.point_range[i]);
        float diff2 = std::fabs(cloud_info.point_range[i+1] - cloud_info.point_range[i]);
        if(diff1 > 0.02 * cloud_info.point_range[i] && diff2 > 0.02 * cloud_info.point_range[i])
            cloudNeighborPicked_[i] = 1;
    }
}

void DataPretreat::extractFeatures(msg_interface::msg::CloudInfo& cloud_info, pcl::PointCloud<PointType>::Ptr &corner_cloud,
                                   pcl::PointCloud<PointType>::Ptr &surface_cloud){
    corner_cloud->clear();
    surface_cloud->clear();

    pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

    for(int i = 0; i < N_SCAN_; i++){
        // 将一条扫描线扫描一周的点云数据，划分为6段，每段分开提取有限数量的特征，保证特征均匀分布
        for(int j = 0; j < 6; j++){
             //假设 当前ring在一维数组中起始点是m，结尾点为n（不包括n），那么6段的起始点分别为：
            // m + [(n-m)/6]*j   j从0～5
            // 化简为 [（6-j)*m + nj ]/6
            // 6段的终止点分别为：
            // m + (n-m)/6 + [(n-m)/6]*j -1  j从0～5,-1是因为最后一个,减去1
            // 化简为 [（5-j)*m + (j+1)*n ]/6 -1
            int sp = (cloud_info.start_ring_index[i] * (6 - j) + cloud_info.end_ring_index[i] * j) / 6;
            int ep = (cloud_info.start_ring_index[i] * (5 - j) + cloud_info.end_ring_index[i] * (j + 1)) / 6 - 1;

            if(sp >= ep){
                LOG(INFO) << "[extractFeatures]: sp >= ep , return ...\n";
                continue;
            }
            // 本段中起始点到终止点的斜率按照从小到大排序
            std::sort(cloud_smoothness_.begin()+sp , cloud_smoothness_.begin()+ep , by_value());

            // 寻找角点
            int cornerCounter = 0;
            for(int k = ep; k >= sp; k--){
                int index = cloud_smoothness_[k].index;
                // 判断曲率是否小于阈值
                if(cloudNeighborPicked_[index] == 0 && cloudCurvature_[index] > edgeThreshold_){
                    if(cornerCounter > 20) break;

                    cloudLabel_[index] = 1;
                    corner_cloud->push_back(extracted_cloud_->at(index));
                    cloudNeighborPicked_[index] = 1;

                    // 将角点周围前后各5个点标记为picked, 防止角点过于密集
                    for (int l = 1; l <= 5; l++){
                        int columnDiff = std::abs(int(cloud_info.point_col_ind[index + l] - cloud_info.point_col_ind[index + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked_[index + l] = 1;
                    }

                    for (int l = -1; l >= -5; l--){
                        int columnDiff = std::abs(int(cloud_info.point_col_ind[index + l] - cloud_info.point_col_ind[index + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked_[index + l] = 1;
                    }
                    cornerCounter++;
                }
            }

            // 添加平面点
            for(int k = sp; k <= ep; k++){
                int index = cloud_smoothness_[k].index;
                if(cloudNeighborPicked_[index] == 0 && cloudCurvature_[index] < surfThreshold_){
                    cloudLabel_[index] = -1;
                    cloudNeighborPicked_[index] = 1;
                    // 同一条扫描线上后5个点标记一下，不再处理，避免特征聚集
                    for (int l = 1; l <= 5; l++){
                        int columnDiff = std::abs(int(cloud_info.point_col_ind[index + l] - cloud_info.point_col_ind[index + l - 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked_[index + l] = 1;
                    }
                    // 同一条扫描线上前5个点标记一下，不再处理，避免特征聚集
                    for (int l = -1; l >= -5; l--){

                        int columnDiff = std::abs(int(cloud_info.point_col_ind[index + l] - cloud_info.point_col_ind[index + l + 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked_[index + l] = 1;
                    }
                }
            }    
            // 平面点和未被处理的点(<=0)，都认为是平面点，加入平面点云集合
            for(int k = sp; k <= ep; k++){
                if(cloudLabel_[k] <= 0)
                    surfaceCloudScan->push_back(extracted_cloud_->at(k));
            }
        }
    }

    // 降采样
    surfaceCloudScanDS->clear();
    voxel_filter_.setInputCloud(surfaceCloudScan);
    voxel_filter_.filter(*surfaceCloudScanDS);
    *surface_cloud = *surfaceCloudScanDS;
}

// 判断是否每个点带时间戳
void DataPretreat::checkPointTime(const sensor_msgs::msg::PointCloud2 &cloud_in){
    // check point time
    if (deskew_flag_ == 0)
    {
        deskew_flag_ = -1;
        for (auto &field : cloud_in.fields)
        {
            if (field.name == "time" || field.name == "t" || field.name == "timestamp")
            {
                deskew_flag_ = 1;
                break;
            }
        }
        if (deskew_flag_ == -1)
            LOG(INFO) << "Point cloud timestamp not available, deskew function disabled, system will drift significantly!\n";
    } 
}

/* 根据外参将imu数据转为雷达坐标系，lio-sam是把imu数据先用imuConverter旋转到雷达系下（但其实还差了个平移，因为速度没有平移转换）
   然后他把雷达数据又根据lidar2Imu反向平移了一下，和转换以后差了个平移的imu数据在“中间系”对齐，
   之后算完又从中间系通过imu2Lidar挪回了雷达系进行publish。
 */
sensor_msgs::msg::Imu DataPretreat::convertImu2Lidar(const sensor_msgs::msg::Imu& imu_data){
    sensor_msgs::msg::Imu imu_trans = imu_data;
    Eigen::Vector3d acc(imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z);
    acc = ext_rot_ * acc;
    imu_trans.linear_acceleration.x = acc.x();
    imu_trans.linear_acceleration.y = acc.y();
    imu_trans.linear_acceleration.z = acc.z();

    Eigen::Vector3d gyr(imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z);
    gyr = ext_rot_ * gyr;
    imu_trans.angular_velocity.x = gyr.x();
    imu_trans.angular_velocity.y = gyr.y();
    imu_trans.angular_velocity.z = gyr.z();

    Eigen::Quaterniond q_from(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z);
    Eigen::Quaterniond q_final;
    if (imu_type_ == 0) {
        q_final = ext_rpy_q_;
    } else if (imu_type_ == 1)
        q_final = q_from * ext_rpy_q_;
    else
        LOG(INFO) << "pls set your imu_type, 0 for 6axis and 1 for 9axis" << std::endl;

    q_final.normalize();
    imu_trans.orientation.w = q_final.w();
    imu_trans.orientation.x = q_final.x();
    imu_trans.orientation.y = q_final.y();
    imu_trans.orientation.z = q_final.z();

    // 轻卡上面的原始imu数据是6轴的，但是组合导航也可以输出姿态信息
    if (q_final.norm() < 0.1){
        LOG(INFO) << "Invalid quaternion, please use a 9-axis IMU!\n";
        rclcpp::shutdown();
    }
    return imu_trans;
}

}
