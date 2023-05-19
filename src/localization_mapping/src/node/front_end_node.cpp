/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-13 17:58:45
 */

#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "localization_mapping/global_defination/global_defination.h"
#include "localization_mapping/front_end/front_end_flow.hpp"
#include "glog/logging.h"

/*
    front_end_node: scan to map的雷达里程计
    输入: cloud info(包含点云时刻imu和odom提供的初始值, 提取的特征点以及特征相关的索引信息), 后端优化后的关键帧(pose, 特征点, 用于构建local map)
    作用: 得到当前帧的odom_pose
    输出: laser odometry
 */

int main(int argc , char* argv[]){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = truck_slam::WORK_SPACE_PATH + "/Log/front_end";
    FLAGS_alsologtostderr = true;

    rclcpp::init(argc , argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 4, true);
    std::shared_ptr<truck_slam::FrontEndFlow> node = std::make_shared<truck_slam::FrontEndFlow>("front_end_node", options);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}