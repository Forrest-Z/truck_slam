#ifndef LOCALIZATION_MAPPING_MODEL_REGISTRATION_INTERFACE_HPP_
#define LOCALIZATION_MAPPING_MODEL_REGISTRATION_INTERFACE_HPP_

#include <eigen3/Eigen/Dense>
#include "localization_mapping/utils/sensor_data/costom_point_type.hpp"


namespace truck_slam{
class RegistrationInterface{
public:
    virtual ~RegistrationInterface() = default;

    virtual bool SetInputTarget(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_target) = 0;
    virtual bool ScanMatch(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_source, 
                          const Eigen::Matrix4f& predict_pose, 
                          pcl::PointCloud<pcl::PointXYZI>::Ptr& result_cloud_ptr,
                          Eigen::Matrix4f& result_pose) = 0;
    virtual float GetFitnessScore() = 0;    
};
}

#endif