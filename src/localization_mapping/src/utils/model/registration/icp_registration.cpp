/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-03-06 16:23:20
 */
#include "localization_mapping/utils/model/registration/icp_registration.hpp"

namespace truck_slam{
ICPRegistration::ICPRegistration(const YAML::Node& node)
                 :icp_ptr_(new pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>()){
    float max_corr_dist = node["max_corr_dist"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    float euc_fitness_eps = node["euc_fitness_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);

}


ICPRegistration::ICPRegistration(float max_corr_dist, float trans_eps, float euc_fitness_eps, int max_iter)
                 :icp_ptr_(new pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>()) {

    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

bool ICPRegistration::SetInputTarget(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_target){
    icp_ptr_->setInputTarget(input_target);
    return true;
}

bool ICPRegistration::ScanMatch(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_source, 
                const Eigen::Matrix4f& predict_pose, 
                pcl::PointCloud<pcl::PointXYZI>::Ptr& result_cloud_ptr,
                Eigen::Matrix4f& result_pose){
    icp_ptr_->setInputSource(input_source);
    icp_ptr_->align(*result_cloud_ptr, predict_pose);
    result_pose = icp_ptr_->getFinalTransformation();

    return true;
}

bool ICPRegistration::ScanMatch(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_source, 
                pcl::PointCloud<pcl::PointXYZI>::Ptr& result_cloud_ptr,
                Eigen::Matrix4f& result_pose){
    icp_ptr_->setInputSource(input_source);
    icp_ptr_->align(*result_cloud_ptr);
    result_pose = icp_ptr_->getFinalTransformation();

    return true;
}

float ICPRegistration::GetFitnessScore(){
    return icp_ptr_->getFitnessScore();
}

bool ICPRegistration::HasConverged(){
 return icp_ptr_->hasConverged();
}


bool ICPRegistration::SetRegistrationParam(float max_corr_dist, float trans_eps, float euc_fitness_eps, int max_iter){
    icp_ptr_->setMaxCorrespondenceDistance(max_corr_dist);
    icp_ptr_->setTransformationEpsilon(trans_eps);
    icp_ptr_->setEuclideanFitnessEpsilon(euc_fitness_eps);
    icp_ptr_->setMaximumIterations(max_iter);
    icp_ptr_->setRANSACIterations(0);

    return true;
}
}