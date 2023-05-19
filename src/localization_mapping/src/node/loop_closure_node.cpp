/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-03-06 10:21:03
 */
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "glog/logging.h"

#include "localization_mapping/loop_closure/loop_closure_flow.hpp"
#include "localization_mapping/global_defination/global_defination.h"

/*
    loop_closure_node
    输入: 当前所有关键帧合集
    作用: 回环检测
    输出: 处于回环的前后两关键帧索引，以及相对pose
 */
int main(int argc, char* argv[]){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = truck_slam::WORK_SPACE_PATH + "/Log/loop_closure";
    FLAGS_alsologtostderr = true;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck_slam::LoopClosureFlow>("loop_closure_node"));
    rclcpp::shutdown();
    return 0;
}