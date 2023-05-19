/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-04-14 10:30:19
 */
#ifndef LOCALIZATION_MAPPING_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_
#define LOCALIZATION_MAPPING_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_

#include <eigen3/Eigen/Dense>
#include <pcl/registration/ndt.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include "localization_mapping/utils/model/registration/registration_interface.hpp"
#include "localization_mapping/utils/sensor_data/costom_point_type.hpp"

namespace truck_slam {
class NDTRegistration: public RegistrationInterface {
  public:
    NDTRegistration(const YAML::Node& node);
    NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

    bool SetInputTarget(const pcl::PointCloud<PointType>::Ptr& input_target) override;
    bool ScanMatch(const pcl::PointCloud<PointType>::Ptr& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   pcl::PointCloud<PointType>::Ptr& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
    float GetFitnessScore() override;
  
  private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

  private:
    pcl::NormalDistributionsTransform<PointType, PointType>::Ptr ndt_ptr_;
};
}

#endif