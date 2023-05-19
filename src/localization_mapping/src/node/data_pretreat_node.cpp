/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-08 11:58:40
 */
#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include "glog/logging.h"
#include "localization_mapping/data_pretreat/data_pretreat_flow.hpp"
#include "localization_mapping/global_defination/global_defination.h"


/*
    data_pretreat_node: 处理原始imu 点云数据, 并去畸变, 提取特征点并整合为cloud_info(自定义消息)输入给前端模块进行匹配
    输入: 原始点云, imu里程计(优化bias后), 原始点云数据
    作用: 点云畸变校正, 为前端激光里程计提供初值, 点云特征(角点, 平面点)提取. 处理时间约50ms(在办公电脑上)
    输出: cloud info(包含点云时刻imu和odom提供的初始值, 提取的特征点以及特征相关的索引信息), 频率同点云(10Hz)
 */
int main(int argc , char* argv[]){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = truck_slam::WORK_SPACE_PATH + "/Log/data_pretreat";
    FLAGS_alsologtostderr = true;
    
    rclcpp::init(argc , argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    // 使用多线程,避免各个回调函数被堵塞,导致数据接受出现问题,
    // 例如: imu的回调频率较高(大于100Hz), 但是雷达的频率只有10Hz, 如果不使用多线程,则受到雷达频率的影响,
    //      imu数据入队的频率将降低,从而导致算法中利用雷达时间戳找imu的时候总是要等imu的数据, 影响效率
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 4, true);       // 线程数量
    std::shared_ptr<truck_slam::DataPretreatFlow> node = 
                        std::make_shared<truck_slam::DataPretreatFlow>("data_pretreat_node", options);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}