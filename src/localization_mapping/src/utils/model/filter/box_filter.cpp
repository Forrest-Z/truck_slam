/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-04-14 10:11:46
 */
#include "glog/logging.h"
#include "localization_mapping/utils/model/filter/box_filter.hpp"
namespace truck_slam {
BoxFilter::BoxFilter(YAML::Node node) {
    size_.resize(6);
    edge_.resize(6);
    origin_.resize(3);

    for (size_t i = 0; i < size_.size(); i++) {
        size_.at(i) = node["box_filter_size"][i].as<float>();
    }
    SetSize(size_);
}

void BoxFilter::SetSize(std::vector<float> size) {
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

void BoxFilter::SetOrigin(std::vector<float> origin) {
    origin_ = origin;
    CalculateEdge();
}

void BoxFilter::CalculateEdge() {
    for (size_t i = 0; i < origin_.size(); ++i) {
        edge_.at(2 * i) = size_.at(2 * i) + origin_.at(i);
        edge_.at(2 * i + 1) = size_.at(2 * i + 1) + origin_.at(i);
    }
}

std::vector<float> BoxFilter::GetEdge() {
    return edge_;
}
} 