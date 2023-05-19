/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-08 13:41:14
 */

#include <chrono>

#include "localization_mapping/data_pretreat/data_pretreat_flow.hpp"
#include "glog/logging.h"

namespace truck_slam{

DataPretreatFlow::DataPretreatFlow(const std::string& node_name, const rclcpp::NodeOptions &options)
                        : rclcpp::Node(node_name, options){
    // node params declare and read params to variables 
    paramsInit();
    // reset parameters
    resetParametes();
    // subscribers and publisher init
    subAndPubInit();
    LOG(INFO) << "*************** Data pretreat node Inited ***************\n";
}

void DataPretreatFlow::paramsInit(){
    // LOG(INFO) <<  "Parameters declare ... \n";
    YAML::Node config_node = YAML::LoadFile(WORK_SPACE_PATH + "/config/config.yaml");
    // topic name
    imuTopic_ = config_node["topic"]["imuTopic"].as<std::string>();
    odomTopic_ = config_node["topic"]["imuOdomTopic"].as<std::string>();
    pointCloudTopic_ = config_node["topic"]["pointCloudTopic"].as<std::string>();
    extractedCloudTopic_ = config_node["topic"]["extractedCloudTopic"].as<std::string>();
    cloudInfoTopic_ = config_node["topic"]["cloudInfoTopic"].as<std::string>();
    cornerTopic_ = config_node["topic"]["cornerTopic"].as<std::string>();
    surfaceTopic_ = config_node["topic"]["surfaceTopic"].as<std::string>();
    // lidar params
    sensor_str_ = config_node["lidar"]["sensor_type"].as<std::string>();
    N_SCAN_ = config_node["lidar"]["N_SCAN"].as<int>();
    Horizon_SCAN_ = config_node["lidar"]["Horizon_SCAN"].as<int>();
    // filter
    box_filter_ptr_ = std::make_shared<BoxFilter<PointXYZIRT>>(config_node["box_filter"]);
}

void DataPretreatFlow::resetParametes(){
    // LOG(INFO) <<  "Parameters and variables reset ... \n";
    range_mat_ = cv::Mat(N_SCAN_, Horizon_SCAN_, CV_32F, cv::Scalar::all(FLT_MAX));
    extracted_cloud_.reset(new pcl::PointCloud<PointType>());
    full_point_cloud_.reset(new pcl::PointCloud<PointType>());
    current_cloud_data_.reset(new pcl::PointCloud<PointXYZIRT>());
    corner_cloud_.reset(new pcl::PointCloud<PointType>());
    surface_cloud_.reset(new pcl::PointCloud<PointType>());
    full_point_cloud_->points.resize(N_SCAN_ * Horizon_SCAN_);
    current_cloud_info_.start_ring_index.assign(N_SCAN_ , 0);
    current_cloud_info_.end_ring_index.assign(N_SCAN_ , 0);
    current_cloud_info_.point_col_ind.assign(N_SCAN_ * Horizon_SCAN_ , 0);
    current_cloud_info_.point_range.assign(N_SCAN_ * Horizon_SCAN_ , 0);
}

void DataPretreatFlow::subAndPubInit(){
    // LOG(INFO) <<  "Subscribers and Publishers init ... \n";
    /*  call back gourp说明:
        MutuallyExclusive；互斥，即这个组别中每时刻只允许1个线程，一个callback在执行时，其他只能等待
        Reentrant；可重入，这个组别中每时刻允许多个线程，一个Callback在执行时，其他callback可开启新的线程
        这样也以为这我们可以有效地对ROS2中的callback程序进行控制。在ROS2的node中默认组别是MutuallyExclusive类型，
        即便使用了multiThreadedExecutor，也依然默认MutuallyExclusive类型
    */
    callback_group_sub1_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    callback_group_sub2_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    callback_group_sub3_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    auto sub_opt1 = rclcpp::SubscriptionOptions();
    auto sub_opt2 = rclcpp::SubscriptionOptions();
    auto sub_opt3 = rclcpp::SubscriptionOptions();

    sub_opt1.callback_group = callback_group_sub1_;
    sub_opt2.callback_group = callback_group_sub2_;
    sub_opt3.callback_group = callback_group_sub3_;

    // init subscribers
    raw_imu_sub_ptr_ = this->create_subscription<sensor_msgs::msg::Imu>(imuTopic_ , 2000, std::bind(&DataPretreatFlow::imuHandler, this, std::placeholders::_1), sub_opt1);
    odom_sub_ptr_ = this->create_subscription<nav_msgs::msg::Odometry>(odomTopic_+"_incremental" , 2000, std::bind(&DataPretreatFlow::odometryHandler, this, std::placeholders::_1), sub_opt2);
    raw_cloud_sub_ptr_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(pointCloudTopic_ , 1000, std::bind(&DataPretreatFlow::cloudHandler, this, std::placeholders::_1), sub_opt3);

    // init publishers
    extracted_cloud_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(extractedCloudTopic_, 100);
    corner_cloud_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cornerTopic_, 100);
    surface_cloud_pub_ptr_ =  this->create_publisher<sensor_msgs::msg::PointCloud2>(surfaceTopic_, 100);
    cloud_info_pub_ptr_ = this->create_publisher<msg_interface::msg::CloudInfo>(cloudInfoTopic_, 100);

    // init data pretreator
    data_pretreat_ptr_ = std::make_shared<DataPretreat>();
}

void DataPretreatFlow::imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_data){
    // convert to lidar frame(no translation)
    sensor_msgs::msg::Imu tmp_imu_data = data_pretreat_ptr_->convertImu2Lidar(*imu_data);
    // add new message to buffer:
    std::lock_guard<std::mutex> lock(imu_mutex_);
    raw_imu_deque_.push_back(tmp_imu_data);
}

void DataPretreatFlow::odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometry_msg){
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_deque_.push_back(*odometry_msg);
}

void DataPretreatFlow::cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_data){
    auto t1 = std::chrono::system_clock::now();
    
    // 1. push sensor_data to deque
    if(!loadData(cloud_data)) return;
    // 2. get valid imu and odom data according to cloud timestamp  
    // 根据当前帧雷达的时间戳，找到时间戳附近的imu和odom数据用于点云运动补偿以及用于前端雷达里程计的初始值
    if(!findValidData()) return;
    // 3. project point cloud to array and deskew point cloud debug projectionCloud需要耗时大约230ms, relase模式下projectionCloud可达到5ms以内， 手动实现了一些api优化了时间 
    projectPointCloud();
    // 4. extration segmented cloud for lidar odometry 耗时2~3ms(办公电脑上)
    cloudExtraction();
    // 5. features extraction debug 模式下耗时40ms左右， realse模式编译优化后大概5ms以内
    featureExtraction();
    // 6. publish cloud to feature extration
    publishData();
    // 7. reset
    resetParametes(); 

    auto t2 = std::chrono::system_clock::now();
    LOG(INFO) << "point cloud pretret cost time : " 
        << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() << " ms.\n" ;
}

bool DataPretreatFlow::loadData(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_data){
    // push to deque
    raw_cloud_deque_.push_back(*cloud_data);
    if(raw_cloud_deque_.size() <= 2){
        LOG(INFO) << "Cloud data is not enough , waiting for more cloud data ...\n";
        return false;
    }
    // get current cloud data
    sensor_msgs::msg::PointCloud2 current_cloud_ros = raw_cloud_deque_.front();
    raw_cloud_deque_.pop_front();
    // convert pointcloud2 to PointXYZIRT
    pointCloudToPointXYZRIT(current_cloud_ros, current_cloud_data_);    
    // 【注】：需要确认rslidar每一帧点云的时间戳是最后一个点，velodyne是第一个点
    // current_cloud_start_timestamp_ = rclcpp::Time(current_cloud_ros.header.stamp).seconds();
    // current_cloud_end_timestamp_ = current_cloud_start_timestamp_ + current_cloud_data_->back().time;
    // rslidar
    current_cloud_start_timestamp_ = current_cloud_data_->points[0].timestamp;
    current_cloud_end_timestamp_ = current_cloud_data_->back().timestamp;
    data_pretreat_ptr_->setStartAndEndTime(current_cloud_start_timestamp_ , current_cloud_end_timestamp_);

    // get cloud frame_id，current cloud data timestamp(include start tim and end time)
    current_cloud_header_.stamp = rclcpp::Time(current_cloud_start_timestamp_ * 1e9);
    current_cloud_header_.frame_id = current_cloud_ros.header.frame_id;

    // check current cloud data is dense 检查是否存在无效点 
    if(!current_cloud_data_->is_dense){
        std::vector<int> map;
        pcl::removeNaNFromPointCloud(*current_cloud_data_, *current_cloud_data_, map);
        // LOG(INFO) << "current cloud data is not dense farmat , please remove NaN points ...\n";
    }
    // 存放未降采样的原始点云
    pcl::toROSMsg(*current_cloud_data_, current_cloud_info_.cloud_raw);
    return true;
}

bool DataPretreatFlow::findValidData(){
    // LOG(INFO) <<  "find valid data of imu and odometry ... \n";
    
    // 上锁避免deuqe中的数据变化
    std::lock_guard<std::mutex> lock_imu(imu_mutex_);
    std::lock_guard<std::mutex> lock_odom(odom_mutex_);
    // make sure IMU data available for the scan imu数据队列中必须覆盖当前雷达帧的时间跨度
    if (raw_imu_deque_.empty()){
        LOG(INFO) << "raw imu deque  is empty(), wait for imu data ... \n";
        return false;
    } 
    if(rclcpp::Time(raw_imu_deque_.front().header.stamp).seconds() > current_cloud_start_timestamp_){ 
        LOG(INFO) << "timestamp of front of imu data queue is late ...\n";
        return false;
    }
    if(rclcpp::Time(raw_imu_deque_.back().header.stamp).seconds() < current_cloud_end_timestamp_){
        LOG(INFO) << "timestamp of back of imu data queue is early ...\n";
        return false;
    }

    // 当前帧对应imu数据处理
    // 1、遍历当前激光帧起止时刻之间的imu数据，初始时刻对应imu的姿态角RPY设为当前帧的初始姿态角
    // 2、用角速度、时间积分，计算每一时刻相对于初始时刻的旋转量，初始时刻旋转设为0
    // 注：imu数据都已经转换到lidar系下了, imu去畸变参数计算
    data_pretreat_ptr_->imuDeskewInfo(current_cloud_info_ , raw_imu_deque_ );
    
    // 当前帧对应imu里程计处理
    // 1、遍历当前激光帧起止时刻之间的imu里程计数据，初始时刻对应imu里程计设为当前帧的初始位姿
    // 2、用起始、终止时刻对应imu里程计，计算相对位姿变换，保存平移增量
    // 注：imu数据都已经转换到lidar系下了, 里程计去畸变参数计算
    // data_pretreat_ptr_->odomDeskewInfo(current_cloud_info_ , odom_deque_);

    return true;
}

void DataPretreatFlow::projectPointCloud(){
    // LOG(INFO) <<  "PointCloud projection begin ... \n";
    data_pretreat_ptr_->projectPointCloud(current_cloud_data_ , range_mat_, full_point_cloud_ , current_cloud_info_.imu_available);
}

void DataPretreatFlow::cloudExtraction(){
    // LOG(INFO) << "cloud extraction start ... \n";
    int count = 0;
    //提取特征的时候，每一行的前5个和最后5个不考虑
    //记录每根扫描线起始第5个激光点在一维数组中的索引
    // 80线激光雷达需要进行抽帧
    for(int i = 0; i < N_SCAN_; i++){
        current_cloud_info_.start_ring_index[i] = count - 1 + 5;
        for(int j = 0; j < Horizon_SCAN_; j++){
            if(range_mat_.at<float>(i , j) != FLT_MAX){
                current_cloud_info_.point_col_ind[count] = j;
                current_cloud_info_.point_range[count] = range_mat_.at<float>(i , j);
                extracted_cloud_->push_back(full_point_cloud_->at(j + i*Horizon_SCAN_));
                count++;
                // LOG(INFO) << "extracted_cloud push point " << i << " " << j << " "<< count << std::endl;
            }
        }
        current_cloud_info_.end_ring_index[i] = count - 1 - 5;
    }
    pcl::toROSMsg(*extracted_cloud_ , current_cloud_info_.cloud_deskewed);
}

void DataPretreatFlow::featureExtraction(){
    data_pretreat_ptr_->featureExtraction(current_cloud_info_ , corner_cloud_ , surface_cloud_);
    data_pretreat_ptr_->resetParameters();
    // LOG(INFO) << "feature extraction finished ... \n";
}

void DataPretreatFlow::publishData(){
    // LOG(INFO) << "publish extracted features and current cloud info ... \n";
    current_cloud_info_.cloud_deskewed.header = current_cloud_header_;
    current_cloud_info_.cloud_corner.header = current_cloud_header_;
    current_cloud_info_.cloud_surface.header = current_cloud_header_;

    // de-skew cloud
    extracted_cloud_pub_ptr_->publish(current_cloud_info_.cloud_deskewed); 
    // save newly extracted features 
    corner_cloud_pub_ptr_->publish(current_cloud_info_.cloud_corner);
    surface_cloud_pub_ptr_->publish(current_cloud_info_.cloud_surface);
    // cloud info after feature extraction
    current_cloud_info_.header = current_cloud_header_;
    cloud_info_pub_ptr_->publish(current_cloud_info_);
}

void DataPretreatFlow::pointCloudToPointXYZRIT(sensor_msgs::msg::PointCloud2 &cloud_in,
                                               pcl::PointCloud<PointXYZIRT>::Ptr &cloud_out){
    if (sensor_str_ == "velodyne" || sensor_str_ == "livox")
    {   
        pcl::moveFromROSMsg(cloud_in, *cloud_out);
        // LOG(INFO) << "cloud size : " << cloud_data.cloud_ptr->points.size();
    }else if (sensor_str_ == "ouster"){
        // Convert to Velodyne format
        pcl::PointCloud<OusterPointXYZIRT>::Ptr tmp_ouster_cloud_in(new pcl::PointCloud<OusterPointXYZIRT>());
        pcl::moveFromROSMsg(cloud_in, *tmp_ouster_cloud_in);
        cloud_out->resize(tmp_ouster_cloud_in->size());
        cloud_out->is_dense = tmp_ouster_cloud_in->is_dense;
        for (size_t i = 0; i < tmp_ouster_cloud_in->size(); i++)
        {
            auto &src = tmp_ouster_cloud_in->at(i);
            auto &dst = cloud_out->at(i);
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.intensity;
            dst.ring = src.ring;
            dst.timestamp = src.t * 1e-9f;
        }
    } else if(sensor_str_ == "rs"){
        pcl::PointCloud<RsPointXYZIRT>::Ptr tmp_rs_cloud_in(new pcl::PointCloud<RsPointXYZIRT>());
        pcl::moveFromROSMsg(cloud_in , *tmp_rs_cloud_in);
        cloud_out->resize(tmp_rs_cloud_in->size());
        cloud_out->is_dense = tmp_rs_cloud_in->is_dense;
        // double start_time = tmp_rs_cloud_in->points[0].timestamp;
        for(size_t i = 0; i < tmp_rs_cloud_in->size(); i++){
            auto &src = tmp_rs_cloud_in->at(i);
            auto &dst = cloud_out->at(i);
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.intensity;
            dst.ring = src.ring;
            dst.timestamp = src.timestamp;
            // LOG(INFO) << "src.time :" << std::to_string(src.timestamp) << std::endl;
            // LOG(INFO) << "dst.time :" << std::to_string(dst.timestamp) << std::endl;
        }
    }else{
        LOG(FATAL) << "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox' or 'rs'): " << sensor_str_ << "\n";
        rclcpp::shutdown();
    }           
    // 去掉雷达后面的车厢点云
    pcl::PointCloud<PointXYZIRT>::Ptr tmp_cloud(new pcl::PointCloud<PointXYZIRT>(*cloud_out));
    box_filter_ptr_->Filter(tmp_cloud, cloud_out, true);

    // 判断是否每个点带时间戳
    data_pretreat_ptr_->checkPointTime(cloud_in);                                
}


} // truck_slam

