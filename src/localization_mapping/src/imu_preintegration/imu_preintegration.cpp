/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-03-06 19:31:29
 */
#include "localization_mapping/imu_preintegration/imu_preintegration.hpp"

namespace truck_slam{
IMUPreIntegration::IMUPreIntegration(const std::string &node_name , const rclcpp::NodeOptions &options) 
                    : Node(node_name, options){
    loadParams();
    initParams();
    subAndPubInit();
    LOG(INFO) << "*************** IMU PreIntegration node Inited ***************\n";
}

void IMUPreIntegration::loadParams(){
    LOG(INFO) <<  "Parameters and variables reset ... \n";
    YAML::Node config_node = YAML::LoadFile(truck_slam::WORK_SPACE_PATH + "/config/config.yaml");
    opt_laser_odom_topic_ = config_node["topic"]["optOdomTopic"].as<std::string>();
    imu_raw_topic_ = config_node["topic"]["imuTopic"].as<std::string>();
    imu_odom_topic_ = config_node["topic"]["imuOdomTopic"].as<std::string>();
    baselink_odom_topic_ = config_node["topic"]["imuBaseOdomTopic"].as<std::string>();

    odometry_frame_ = config_node["frame"]["odometryFrame"].as<std::string>();
    laser_frame_ = config_node["frame"]["laserFrame"].as<std::string>();
    base_link_frame_ = config_node["frame"]["baselinkFrame"].as<std::string>();
    imu_frame_ = config_node["frame"]["imuFrame"].as<std::string>();

    imu_to_lidar_vec_ = config_node["extrinsics"]["imu_to_lidar"].as<std::vector<float>>();
    baselink_to_imu_vec_ = config_node["extrinsics"]["baselink_to_imu"].as<std::vector<float>>();

    imu_acc_noise_ = config_node["imu"]["imuAccNoise"].as<float>();
    imu_gyr_noise_ = config_node["imu"]["imuGyrNoise"].as<float>(); 
    imu_acc_biasN_ = config_node["imu"]["imuAccBiasN"].as<float>();
    imu_gyr_biasN_ = config_node["imu"]["imuGyrBiasN"].as<float>();
    imu_gravity_   = config_node["imu"]["imuGravity"].as<float>();
    imu_freq_      = config_node["imu"]["imuFrequency"].as<int>();
    imu_type_      = config_node["imu"]["imuType"].as<int>();
}

void IMUPreIntegration::initParams(){
    // LOG(INFO) <<  "Subscribers, Publishers Init ... \n";
    tf2::Quaternion q; q.setRPY(-baselink_to_imu_vec_[0], -baselink_to_imu_vec_[1], -baselink_to_imu_vec_[2]);
    tf2::Vector3 t(-baselink_to_imu_vec_[3], -baselink_to_imu_vec_[4], -baselink_to_imu_vec_[5]);
    imu_to_baselink_.setRotation(q); imu_to_baselink_.setOrigin(t);

    prior_pose_noise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished()); // rad,rad,rad,m, m, m
    prior_vel_noise_   = gtsam::noiseModel::Isotropic::Sigma(3, 1e-4); // m/s
    prior_bias_noise_  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
    correction_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished()); // rad,rad,rad,m, m, m
    noise_model_between_bias_ = (gtsam::Vector(6) << imu_acc_biasN_, imu_acc_biasN_, imu_acc_biasN_, imu_gyr_biasN_, imu_gyr_biasN_, imu_gyr_biasN_).finished();

    std::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imu_gravity_);
    p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imu_acc_noise_, 2); // acc white noise in continuous
    p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imu_gyr_noise_, 2); // gyro white noise in continuous
    p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias
    imu_integrator_imu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
    imu_integrator_opt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        
    lidar_to_imu_ = gtsam::Pose3(gtsam::Rot3::RzRyRx(-imu_to_lidar_vec_[0], -imu_to_lidar_vec_[1], -imu_to_lidar_vec_[2]), gtsam::Point3(-imu_to_lidar_vec_[3], -imu_to_lidar_vec_[4], -imu_to_lidar_vec_[5]));
    done_first_opt_ = false;
    system_inited_ = false;
    last_imu_time_opt_ = -1;
    last_imu_time_ = -1;
}

void IMUPreIntegration::subAndPubInit(){
    callback_group1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub_opt1 = rclcpp::SubscriptionOptions();
    auto sub_opt2 = rclcpp::SubscriptionOptions();
    sub_opt1.callback_group = callback_group1_;
    sub_opt2.callback_group = callback_group2_;
    imu_raw_sub_ptr_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_raw_topic_, 1000, 
                        std::bind(&IMUPreIntegration::IMUCallback, this, std::placeholders::_1), sub_opt1);
    opt_laser_odom_sub_ptr_ = this->create_subscription<nav_msgs::msg::Odometry>(opt_laser_odom_topic_, 1000,
                        std::bind(&IMUPreIntegration::optLaserOdomCallback, this, std::placeholders::_1), sub_opt2);
    imu_odom_pub_ptr_ = this->create_publisher<nav_msgs::msg::Odometry>(imu_odom_topic_, 1000);
    baselink_odom_pub_ptr_ = this->create_publisher<nav_msgs::msg::Odometry>(baselink_odom_topic_, 1000);

    tf_odom_to_imu_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_odom_to_baselink_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void IMUPreIntegration::resetOptimization(){
    gtsam::ISAM2Params opt_parameters_;
    opt_parameters_.setRelinearizeThreshold(0.1);
    opt_parameters_.relinearizeSkip = 1;
    optimizer_ = gtsam::ISAM2(opt_parameters_);
    gtsam::NonlinearFactorGraph new_graph_factors;
    graph_factor_ = new_graph_factors;

    gtsam::Values new_vaules;
    graph_values_ = new_vaules;
}

void IMUPreIntegration::resetParams(){
    last_imu_time_ = -1;
    done_first_opt_ = false;
    system_inited_ = false;
}

void IMUPreIntegration::optLaserOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg){
    std::lock_guard<std::mutex> lock(laser_odom_mtx_);
    double current_odom_time = rclcpp::Time(odom_msg->header.stamp).seconds();

    // 检查imu队列是否为空
    if(imu_que_opt_.empty())
        return;
            
    // 当前帧激光位姿，来自scan-to-map匹配、因子图优化后的位姿
    float p_x = odom_msg->pose.pose.position.x;
    float p_y = odom_msg->pose.pose.position.y;
    float p_z = odom_msg->pose.pose.position.z;
    float r_w = odom_msg->pose.pose.orientation.w;
    float r_x = odom_msg->pose.pose.orientation.x;
    float r_y = odom_msg->pose.pose.orientation.y;
    float r_z = odom_msg->pose.pose.orientation.z;
    // ! 此处未考虑退化问题
    bool degenerate = (int)odom_msg->pose.covariance[0] == 1 ? true : false;
    gtsam::Pose3 lidar_pose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

    // 0. 系统初始化，第一帧
    if(!system_inited_){
        LOG(INFO) << "imu PreIntegration System Inited ... " << std::endl;
        // 重置ISAM2优化器
        resetOptimization();
        // 将imu deque opt 早于第一帧的数据都清掉
        while(!imu_que_opt_.empty()){
            double tmp_time = rclcpp::Time(imu_que_opt_.front().header.stamp).seconds();  
            if(last_imu_time_opt_ < current_odom_time - delta_t_){
                last_imu_time_opt_ = tmp_time;
                imu_que_opt_.pop_front();
            }else {
                break;
            }
        }

        // lidarPose 为本回调函数收到的激光里程计数据，重组成gtsam的pose格式并转到imu坐标系下
        prev_pose_ = lidar_pose.compose(lidar_to_imu_);
        gtsam::PriorFactor<gtsam::Pose3> prior_pose(X(0), prev_pose_, prior_pose_noise_);
        graph_factor_.add(prior_pose);

        prev_vel_ = gtsam::Vector3(0, 0, 0);
        gtsam::PriorFactor<gtsam::Vector3> prior_vel(V(0), prev_vel_, prior_vel_noise_);
        graph_factor_.add(prior_vel);

        prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
        prev_bias_ = gtsam::imuBias::ConstantBias();
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> prior_bias(B(0), prev_bias_, prior_bias_noise_);
        graph_factor_.add(prior_bias);

        // 变量节点赋初值
        graph_values_.insert(X(0), prev_pose_);
        graph_values_.insert(V(0), prev_vel_);
        graph_values_.insert(B(0), prev_bias_);


        // 优化一次
        optimizer_.update(graph_factor_, graph_values_);
        //图和节点均清零  为什么要清零不能继续用吗?
        //是因为节点信息保存在gtsam::ISAM2 optimizer，所以要清理后才能继续使用
        graph_factor_.resize(0);
        graph_values_.clear();

        //积分器重置,重置优化之后的偏置
        imu_integrator_imu_->resetIntegrationAndSetBias(prev_bias_);
        imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);
        counter_ = 1;
        system_inited_ = true;
        LOG(INFO) << "imu pre_integration is inited ... \n";
        return;
    }

    // 每隔100帧激光里程计，重置ISAM2优化器，保证优化效率
    if(counter_ == 100){
        // 前一帧的位姿、速度、偏置噪声模型 
        gtsam::noiseModel::Gaussian::shared_ptr update_pose_noise = gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(X(counter_ - 1)));
        gtsam::noiseModel::Gaussian::shared_ptr update_vel_noise = gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(V(counter_ - 1)));
        gtsam::noiseModel::Gaussian::shared_ptr update_bias_noise = gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(B(counter_ - 1)));
        resetOptimization();

        // 将前一帧的状态添加里程计位姿作为先验因子
        gtsam::PriorFactor<gtsam::Pose3> prior_pose(X(0), prev_pose_, update_pose_noise);
        graph_factor_.add(prior_pose);
        
        gtsam::PriorFactor<gtsam::Vector3> prior_vel(V(0), prev_vel_, update_vel_noise);
        graph_factor_.add(prior_vel);

        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> prev_bias(B(0), prev_bias_, update_bias_noise);
        graph_factor_.add(prev_bias);
        graph_values_.insert(X(0), prev_pose_);
        graph_values_.insert(V(0), prev_vel_);
        graph_values_.insert(B(0), prev_bias_);

        optimizer_.update(graph_factor_, graph_values_);
        graph_factor_.resize(0);
        graph_values_.clear();

        counter_ = 1;
    }

    LOG(INFO) << "find valid imu data ... " << std::endl;
    // 1. 计算前一帧与当前帧之间的imu预积分量，用前一帧状态施加预积分量得到当前帧初始状态估计，
    //  添加来自back end的当前帧位姿，进行因子图优化，更新当前帧状态
    while(!imu_que_opt_.empty()){
        sensor_msgs::msg::Imu *this_imu_data = &imu_que_opt_.front();
        double imu_time = rclcpp::Time(this_imu_data->header.stamp).seconds();
        if(imu_time < current_odom_time - delta_t_){
            // 计算imu数据时间相对于上一帧时间， 用于预积分
            // ! 注意这个500 是imu的频率根据实际进行调整
            double dt = (last_imu_time_opt_ < 0)? (1.0 / imu_freq_) : (imu_time - last_imu_time_opt_); 
            imu_integrator_opt_->integrateMeasurement(
                gtsam::Vector3(this_imu_data->linear_acceleration.x, this_imu_data->linear_acceleration.y, this_imu_data->linear_acceleration.z),
                gtsam::Vector3(this_imu_data->angular_velocity.x, this_imu_data->angular_velocity.y, this_imu_data->angular_velocity.z), dt
            );
            last_imu_time_opt_ = imu_time;
            imu_que_opt_.pop_front();
        }else
            break;
    }

    LOG(INFO) << "start to imu preintegration optimization ... " << std::endl;
    //利用两帧之间的IMU数据完成了预积分后增加imu因子到因子图中
    const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imu_integrator_opt_);
    gtsam::ImuFactor imu_factor(X(counter_-1), V(counter_-1), X(counter_), V(counter_), B(counter_-1), preint_imu);
    graph_factor_.add(imu_factor);
    // add imu bias between factor
    graph_factor_.add(
        gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(counter_-1), B(counter_), gtsam::imuBias::ConstantBias(),
        gtsam::noiseModel::Diagonal::Sigmas(std::sqrt(imu_integrator_opt_->deltaTij()) * noise_model_between_bias_)));
    
    // add pose factor
    gtsam::Pose3 current_pose = lidar_pose.compose(lidar_to_imu_); 
    // 用前一帧的状态、偏置，施加imu预计分量，得到当前帧的状态
    gtsam::NavState prop_state = imu_integrator_opt_->predict(prev_state_, prev_bias_);
    gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(counter_), current_pose, correction_noise_);
    graph_factor_.add(pose_factor);

    graph_values_.insert(X(counter_), prop_state.pose());
    graph_values_.insert(V(counter_), prop_state.v());
    graph_values_.insert(B(counter_), prev_bias_);


    // 执行优化
    optimizer_.update(graph_factor_, graph_values_);
    optimizer_.update();
    graph_factor_.resize(0);
    graph_values_.clear();

    // 得到当前帧的最新状态
    gtsam::Values current_opt_result = optimizer_.calculateBestEstimate();
    prev_pose_ = current_opt_result.at<gtsam::Pose3>(X(counter_));
    prev_vel_ = current_opt_result.at<gtsam::Vector3>(V(counter_));
    prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
    prev_bias_ = current_opt_result.at<gtsam::imuBias::ConstantBias>(B(counter_));

    LOG(INFO) << "resetIntegrationAndSetBias ... " << std::endl;
    // 根据最新的优化得到的bias重新预积分
    imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);

    LOG(INFO) << "check optimization ... " << std::endl;
    // check optimization
    if (failureDetection(prev_vel_, prev_bias_)){
        resetParams();
        return;
    }
    
    // 用于imu odom 预积分
    prev_state_odom_ = prev_state_;
    prev_bias_odom_ = prev_bias_;

    // 用最新的偏置重新计算当前激光里程计时刻之后的imu预积分，这个预积分用于计算每时刻位姿 
    double last_imu_time = -1;

    // 将imu队列中早于当前帧的数据pop
    while(!imu_que_imu_.empty()){
        double tmp_time = rclcpp::Time(imu_que_imu_.front().header.stamp).seconds();
        if(tmp_time < current_odom_time - delta_t_){
            last_imu_time = tmp_time;
            imu_que_imu_.pop_front();
        }else{
            break;
        }
    }

    LOG(INFO) << "update PreIntegration ... " << std::endl;
    // 重新预积分
    if (!imu_que_imu_.empty()){
        imu_integrator_imu_->resetIntegrationAndSetBias(prev_bias_odom_);
        for (int i = 0; i < (int)imu_que_imu_.size(); ++i){
            sensor_msgs::msg::Imu this_imu = imu_que_imu_[i];
            double imu_time = rclcpp::Time(this_imu.header.stamp).seconds();
            // ! 注意这个500 是imu的频率根据实际进行调整
            double dt = (last_imu_time < 0) ? (1.0 / imu_freq_) : (imu_time - last_imu_time);
            imu_integrator_imu_->integrateMeasurement(
                gtsam::Vector3(this_imu.linear_acceleration.x, this_imu.linear_acceleration.y, this_imu.linear_acceleration.z),
                gtsam::Vector3(this_imu.angular_velocity.x, this_imu.angular_velocity.y, this_imu.angular_velocity.z), dt);

            last_imu_time = imu_time;
        }
    }

    counter_++;
    done_first_opt_ = true;
}

bool IMUPreIntegration::failureDetection(const gtsam::Vector3& vel_cur, const gtsam::imuBias::ConstantBias& bias_cur){
    Eigen::Vector3f vel(vel_cur.x(), vel_cur.y(), vel_cur.z());
    if (vel.norm() > 30)
    {
        LOG(INFO) << "Large velocity, reset IMU-preintegration!\n";
        return true;
    }

    Eigen::Vector3f ba(bias_cur.accelerometer().x(), bias_cur.accelerometer().y(), bias_cur.accelerometer().z());
    Eigen::Vector3f bg(bias_cur.gyroscope().x(), bias_cur.gyroscope().y(), bias_cur.gyroscope().z());
    if (ba.norm() > 1.0 || bg.norm() > 1.0)
    {
        LOG(INFO) << "Large bias, reset IMU-preintegration!\n";
        return true;
    }

    return false;
}

void IMUPreIntegration::IMUCallback(const sensor_msgs::msg::Imu::SharedPtr imu_raw){
    std::lock_guard<std::mutex> lock(imu_mtx_);
    sensor_msgs::msg::Imu current_imu_data = *imu_raw;
    imu_que_imu_.push_back(current_imu_data);
    imu_que_opt_.push_back(current_imu_data);

    // 如果第一次优化还没有完成则不做处理
    if(!done_first_opt_)
        return;
    
    double current_imu_time = rclcpp::Time(current_imu_data.header.stamp).seconds();
    // ! 注意这个500 是imu的频率根据实际进行调整
    double dt = (last_imu_time_ < 0) ? (1.0 / imu_freq_) : (current_imu_time - last_imu_time_);
    last_imu_time_ = current_imu_time;

    // 将当前数据进行预积分
    imu_integrator_imu_->integrateMeasurement(gtsam::Vector3(current_imu_data.linear_acceleration.x, current_imu_data.linear_acceleration.y, current_imu_data.linear_acceleration.z),
                                              gtsam::Vector3(current_imu_data.angular_velocity.x, current_imu_data.angular_velocity.y, current_imu_data.angular_velocity.z), dt);
    
    // 根据上一帧到当前帧之间优化得到的状态以及imu\bias， 预测当前的状态
    gtsam::NavState current_state = imu_integrator_imu_->predict(prev_state_odom_, prev_bias_odom_);

    // 发布预测后的位置和姿态(imu frame)
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = current_imu_data.header.stamp;
    odometry.header.frame_id = odometry_frame_;
    odometry.child_frame_id = "odom_imu";
    gtsam::Pose3 imu_pose = gtsam::Pose3(current_state.quaternion(), current_state.position());
    transToOdom(imu_pose, odometry);
    odometry.twist.twist.linear.x = current_state.velocity().x();
    odometry.twist.twist.linear.y = current_state.velocity().y();
    odometry.twist.twist.linear.z = current_state.velocity().z();
    odometry.twist.twist.angular.x = current_imu_data.angular_velocity.x + prev_bias_odom_.gyroscope().x();
    odometry.twist.twist.angular.y = current_imu_data.angular_velocity.y + prev_bias_odom_.gyroscope().y();
    odometry.twist.twist.angular.z = current_imu_data.angular_velocity.z + prev_bias_odom_.gyroscope().z();
    imu_odom_pub_ptr_->publish(odometry);

    // 发布预测后的位置和姿态(baselink frame)
    nav_msgs::msg::Odometry base_odometry;
    base_odometry.header.stamp = current_imu_data.header.stamp;
    base_odometry.header.frame_id = odometry_frame_;
    base_odometry.child_frame_id = "odom_baselink";
    gtsam::Pose3 baselink_pose = imu_pose * gtsam::Pose3(gtsam::Rot3::RzRyRx(-baselink_to_imu_vec_[0], -baselink_to_imu_vec_[1], -baselink_to_imu_vec_[2]),
                                                         gtsam::Point3(-baselink_to_imu_vec_[3], -baselink_to_imu_vec_[4], -baselink_to_imu_vec_[5]));
    transToOdom(baselink_pose, base_odometry);
    gtsam::Rot3 baselink_to_imu(gtsam::Rot3::RzRyRx(baselink_to_imu_vec_[0], baselink_to_imu_vec_[1], baselink_to_imu_vec_[2]));
    // 速度转为baselink
    gtsam::Point3 linear_vel(odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z);
    gtsam::Point3 angular_vel(odometry.twist.twist.angular.x, odometry.twist.twist.angular.y, odometry.twist.twist.angular.z);
    linear_vel = baselink_to_imu * linear_vel;
    angular_vel = baselink_to_imu * angular_vel;
    base_odometry.twist.twist.linear.x  = linear_vel.x();
    base_odometry.twist.twist.linear.y  = linear_vel.y();
    base_odometry.twist.twist.linear.z  = linear_vel.z();
    base_odometry.twist.twist.angular.x = angular_vel.x();
    base_odometry.twist.twist.angular.y = angular_vel.y();
    base_odometry.twist.twist.angular.z = angular_vel.z();
    baselink_odom_pub_ptr_->publish(base_odometry);

    // publish odom to imu tf
    tf2::Transform tCur;
    tf2::fromMsg(odometry.pose.pose, tCur);
    geometry_msgs::msg::TransformStamped odom_to_imu;
    odom_to_imu.header.stamp    = odometry.header.stamp;
    odom_to_imu.header.frame_id = odometry_frame_; 
    odom_to_imu.child_frame_id  = imu_frame_;
    odom_to_imu.transform       = tf2::toMsg(tCur);
    tf_odom_to_imu_->sendTransform(odom_to_imu);

    // publish odom to baselink tf
    tCur *= imu_to_baselink_;
    geometry_msgs::msg::TransformStamped odom_to_baselink;
    odom_to_baselink.header.stamp    = odometry.header.stamp;
    odom_to_baselink.header.frame_id = odometry_frame_; 
    odom_to_baselink.child_frame_id  = base_link_frame_;
    odom_to_baselink.transform       = tf2::toMsg(tCur);
    tf_odom_to_baselink_->sendTransform(odom_to_baselink);
}
}