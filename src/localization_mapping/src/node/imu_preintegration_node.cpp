/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-03-06 21:14:01
 */
#include <rclcpp/rclcpp.hpp>
#include "localization_mapping/global_defination/global_defination.h"
#include "localization_mapping/imu_preintegration/imu_preintegration.hpp"
#include "glog/logging.h"

/*
    imu_preintegration_node: 
    输入: 后端优化得到的雷达里程计（这个里程计是当前帧与前一帧增量，作用在前一帧得到当前帧的里程计，并且和imu作了加权，优化了pitch和roll）
    作用: 预积分得到高频率imu odometry
    输出: imu odometry（频率同imu）
 */
int main(int argc, char* argv[]){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = truck_slam::WORK_SPACE_PATH + "/Log/imu_preintegration";
    FLAGS_alsologtostderr = true;

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 4, true);
    std::shared_ptr<truck_slam::IMUPreIntegration> node = std::make_shared<truck_slam::IMUPreIntegration>("imu_pre_integration_node", options);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}