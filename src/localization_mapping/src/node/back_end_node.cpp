/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-23 11:54:20
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "localization_mapping/back_end/back_end_flow.hpp"
#include "localization_mapping/global_defination/global_defination.h"
#include "glog/logging.h"

/*
    back_end_node: mapping and optimization of laser odometry
    输入: 前端里程计匹配的pose， GPS先验， loop closure约束
    作用: 建图，融合图优化得到odometry
    输出: odom_mapping
 */
int main(int argc, char* argv[]){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = truck_slam::WORK_SPACE_PATH + "/Log/back_end";
    FLAGS_alsologtostderr = true;

    rclcpp::init(argc , argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 4, true);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    std::shared_ptr<truck_slam::BackEndFlow> node = std::make_shared<truck_slam::BackEndFlow>("back_end_node", options);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}