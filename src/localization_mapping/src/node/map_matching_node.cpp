/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-13 17:58:45
 */

#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "localization_mapping/global_defination/global_defination.h"
#include "localization_mapping/map_matching/map_matching_flow.hpp"
#include "glog/logging.h"

/*
    map_matching_node: 基于地图的定位
    输入: cloud info(包含点云时刻imu和odom提供的初始值, 提取的特征点以及特征相关的索引信息), 已知地图
    作用: scan to map 
    输出: 匹配后的位姿 
 */

int main(int argc , char* argv[]){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = truck_slam::WORK_SPACE_PATH + "/Log/map_matching";
    FLAGS_alsologtostderr = true;

    rclcpp::init(argc , argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 4, true);
    std::shared_ptr<truck_slam::MapMatchingFlow> node = std::make_shared<truck_slam::MapMatchingFlow>("map_matching_node", options);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}