/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-04-14 10:07:12
 */
#ifndef LOCALIZATION_MAPPING_MODELS_CLOUD_FILTER_BOX_FILTER_HPP_
#define LOCALIZATION_MAPPING_MODELS_CLOUD_FILTER_BOX_FILTER_HPP_

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include "localization_mapping/utils/sensor_data/costom_point_type.hpp"

namespace truck_slam {    
template <typename PointT>
class BoxFilter{
public:
    BoxFilter(YAML::Node node){
        size_.resize(6);
        edge_.resize(6);
        origin_.resize(3);
        for (size_t i = 0; i < size_.size(); i++) 
            size_.at(i) = node["box_filter_size"][i].as<float>();
        SetSize(size_);
    }

    BoxFilter() = default;

    bool Filter(const typename pcl::PointCloud<PointT>::Ptr& input_cloud_ptr, typename pcl::PointCloud<PointT>::Ptr& output_cloud_ptr, bool negative = false) {
        output_cloud_ptr->clear();
        pcl_box_filter_.setMin(Eigen::Vector4f(edge_.at(0), edge_.at(2), edge_.at(4), 1.0e-6));
        pcl_box_filter_.setMax(Eigen::Vector4f(edge_.at(1), edge_.at(3), edge_.at(5), 1.0e6));
        pcl_box_filter_.setInputCloud(input_cloud_ptr);
        pcl_box_filter_.setNegative(negative);      // 设置去掉的是范围内（true）还是范围外（false）, 默认false
        pcl_box_filter_.filter(*output_cloud_ptr);
        return true;
    }

    void SetSize(std::vector<float> size){
        size_ = size;
        std::cout << "Box Filter params:" << std::endl
              << "min_x: " << size.at(0) << ", "
              << "max_x: " << size.at(1) << ", "
              << "min_y: " << size.at(2) << ", "
              << "max_y: " << size.at(3) << ", "
              << "min_z: " << size.at(4) << ", "
              << "max_z: " << size.at(5)
              << std::endl << std::endl;
        CalculateEdge();
    }

    void SetOrigin(std::vector<float> origin){
        origin_ = origin;
        CalculateEdge();
    }

    std::vector<float> GetEdge() { return edge_; }

private:
    void CalculateEdge(){
        for (size_t i = 0; i < origin_.size(); ++i) {
            edge_.at(2 * i) = size_.at(2 * i) + origin_.at(i);
            edge_.at(2 * i + 1) = size_.at(2 * i + 1) + origin_.at(i);
        }
    }
    pcl::CropBox<PointT> pcl_box_filter_;
    std::vector<float> origin_;
    std::vector<float> size_;
    std::vector<float> edge_;
};
}
#endif 