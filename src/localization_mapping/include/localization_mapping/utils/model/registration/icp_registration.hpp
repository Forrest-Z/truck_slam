/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-03-06 15:44:06
 */
#ifndef LOCALIZATION_MAPPING_MODEL_ICP_REGISTRATION_HPP_
#define LOCALIZATION_MAPPING_MODEL_ICP_REGISTRATION_HPP_

#include <yaml-cpp/yaml.h>
#include <pcl/registration/icp.h>
#include "localization_mapping/utils/model/registration/registration_interface.hpp"

namespace truck_slam{
class ICPRegistration : public RegistrationInterface{
public:
    ICPRegistration(const YAML::Node& node);
    ICPRegistration(float res, float step_size, float trans_eps, int max_iter);

    bool SetInputTarget(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_target) override;
    bool ScanMatch(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   pcl::PointCloud<pcl::PointXYZI>::Ptr& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;

    bool ScanMatch(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_source, 
                   pcl::PointCloud<pcl::PointXYZI>::Ptr& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose);
    float GetFitnessScore() override;   
    bool HasConverged();

private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr icp_ptr_;
};
}
#endif