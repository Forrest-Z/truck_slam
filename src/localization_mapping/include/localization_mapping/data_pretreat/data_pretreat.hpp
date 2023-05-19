/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-11 22:11:54
 */
#ifndef LOCALIZATION_MAPPING_DATA_PRETREAT_HPP_
#define LOCALIZATION_MAPPING_DATA_PRETREAT_HPP_

#include <deque>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/impl/crop_box.hpp> 
#include <pcl_conversions/pcl_conversions.h>

#include <opencv4/opencv2/opencv.hpp>

#include "msg_interface/msg/cloud_info.hpp"
#include "localization_mapping/global_defination/global_defination.h"
#include "localization_mapping/utils/tools/tools.hpp"
#include "localization_mapping/utils/sensor_data/costom_point_type.hpp"
#include "glog/logging.h"


namespace truck_slam{

class DataPretreat {
    public:
        DataPretreat();
        void setStartAndEndTime(const double &start_time , const double &end_time);
        void loadParameters();
        void resetParameters();
        void imuDeskewInfo(msg_interface::msg::CloudInfo& cloud_info , std::deque<sensor_msgs::msg::Imu>& imu_deque);
        void odomDeskewInfo(msg_interface::msg::CloudInfo& cloud_info , std::deque<nav_msgs::msg::Odometry>& odom_deque);
        void projectPointCloud(const pcl::PointCloud<PointXYZIRT>::Ptr& current_cloud , cv::Mat& rangeMat, 
                               pcl::PointCloud<PointType>::Ptr& full_cloud, const bool &imu_available);

        void featureExtraction(msg_interface::msg::CloudInfo& cloud_info , pcl::PointCloud<PointType>::Ptr &corner_cloud,
                               pcl::PointCloud<PointType>::Ptr &surface_cloud);

        void calculateSmoothness(msg_interface::msg::CloudInfo& cloud_info);
        void markOccludedPoints(msg_interface::msg::CloudInfo& cloud_info);
        void extractFeatures(msg_interface::msg::CloudInfo& cloud_info, pcl::PointCloud<PointType>::Ptr &corner_cloud,
                             pcl::PointCloud<PointType>::Ptr &surface_cloud);
        sensor_msgs::msg::Imu convertImu2Lidar(const sensor_msgs::msg::Imu& imu_data);

        void findRotation(const double& point_time, float *rotXCur, float *rotYCur, float *rotZCur);
        void findPosition(const double& rel_time, float *posXCur, float *posYCur, float *posZCur);
        void deskewPoint(PointType &point, const double &relTime);
        void checkPointTime(const sensor_msgs::msg::PointCloud2 &cloud_in);
        

    private:
        struct smoothness_t{
            float value;
            size_t index;
        };

        struct by_value{ 
            bool operator()(smoothness_t const &left, smoothness_t const &right) { 
                return left.value < right.value;
            }
        };

        // raw imu data in current cloud timestamp
        int imu_index_; 
        int queue_length_ = 1000;
        double *imuTime_ = new double[queue_length_];
        double *imuRotX_ = new double[queue_length_];
        double *imuRotY_ = new double[queue_length_];
        double *imuRotZ_ = new double[queue_length_];
        // start and end ime of this scan
        double start_time_;
        double end_time_;
        std::vector<int> columnIdnCountVec_;
        float rollIncre_, pitchIncre_, yawIncre_;
        float odomIncreX_, odomIncreY_, odomIncreZ_;
        SensorType sensor_type_;
        int N_SCAN_;
        int Horizon_SCAN_;
        int downsampleRate_;
        double lidarMinRange_;
        double lidarMaxRange_;
        float odometrySurfLeafSize_;
        int imu_type_ = 0;
        pcl::PointCloud<PointType>::Ptr extracted_cloud_;
        std::vector<smoothness_t> cloud_smoothness_;
        float *cloudCurvature_;
        int *cloudNeighborPicked_;
        int *cloudLabel_;
        float edgeThreshold_;
        float surfThreshold_;
        int edgeFeatureMinValidNum_;
        int surfFeatureMinValidNum_;
        pcl::VoxelGrid<PointType> voxel_filter_;
        std::vector<float> imu_to_lidar_vec_;
        Eigen::Matrix3d ext_rot_;
        Eigen::Matrix3d ext_rpy_;
        Eigen::Vector3d ext_trans_;
        Eigen::Quaterniond ext_rpy_q_;
        bool first_point_flag_;
        float trans_start_[6];
        int deskew_flag_ = 0;
};
}

#endif