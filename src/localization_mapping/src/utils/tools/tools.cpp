/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-11 22:58:01
 */

#include "localization_mapping/utils/tools/tools.hpp"

namespace truck_slam{

double getRosTime(const builtin_interfaces::msg::Time &stamp){
    return (static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9);
}

float pointDistance(PointType p1, PointType p2){
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

float pointRange(PointType p){
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

void savePose(std::ofstream &ofs, const Eigen::Matrix4f &pose){
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ofs << pose(i, j);
            if (i == 2 && j == 3) {
                ofs << std::endl;
            } else {
                ofs << " ";
            }
        }
    }
}
pcl::PointCloud<pcl::PointXYZI>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointXYZI> cloudIn, PointXYZIRPYT* transformIn){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>());

    int cloudSize = cloudIn.size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transform = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    #pragma omp parallel for num_threads(8)
    for(int i = 0; i < cloudSize; i++){
        const auto& pointFrom = cloudIn.points[i];
        cloudOut->points[i].x = transform(0 , 0) * pointFrom.x + transform(0 , 1) * pointFrom.y + transform(0 , 2) * pointFrom.z + transform(0 , 3);
        cloudOut->points[i].y = transform(1 , 0) * pointFrom.x + transform(1 , 1) * pointFrom.y + transform(1 , 2) * pointFrom.z + transform(1 , 3);
        cloudOut->points[i].z = transform(2 , 0) * pointFrom.x + transform(2 , 1) * pointFrom.y + transform(2 , 2) * pointFrom.z + transform(2 , 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}


gtsam::Pose3 transToPose3(const nav_msgs::msg::Odometry &pose_data){
    gtsam::Rot3 pose_ori(pose_data.pose.pose.orientation.w, 
                          pose_data.pose.pose.orientation.x,
                          pose_data.pose.pose.orientation.y,
                          pose_data.pose.pose.orientation.z);
    gtsam::Point3 pose_pos(pose_data.pose.pose.position.x,
                            pose_data.pose.pose.position.y,
                            pose_data.pose.pose.position.z);

    return gtsam::Pose3(pose_ori, pose_pos);
}

gtsam::Pose3 transToPose3(const msg_interface::msg::KeyFramePose &pose_data){
     return gtsam::Pose3(gtsam::Rot3::RzRyRx(pose_data.roll, pose_data.pitch, pose_data.yaw),
                         gtsam::Point3(pose_data.x, pose_data.y, pose_data.z));
}

gtsam::Pose3 transToPose3(const PointTypePose &pose_data){
     return gtsam::Pose3(gtsam::Rot3::RzRyRx(pose_data.roll, pose_data.pitch, pose_data.yaw),
                         gtsam::Point3(pose_data.x, pose_data.y, pose_data.z));
}

void transToOdom(const gtsam::Pose3 &pose_data, nav_msgs::msg::Odometry &odom_data){
    odom_data.pose.pose.position.x = pose_data.translation().x();
    odom_data.pose.pose.position.y = pose_data.translation().y();
    odom_data.pose.pose.position.z = pose_data.translation().z();
    odom_data.pose.pose.orientation.w = pose_data.rotation().toQuaternion().w();
    odom_data.pose.pose.orientation.x = pose_data.rotation().toQuaternion().x();
    odom_data.pose.pose.orientation.y = pose_data.rotation().toQuaternion().y();
    odom_data.pose.pose.orientation.z = pose_data.rotation().toQuaternion().z();
}

void transToOdom(const msg_interface::msg::KeyFramePose &pose_data, nav_msgs::msg::Odometry &odom_data){
    odom_data.pose.pose.position.x = pose_data.x;
    odom_data.pose.pose.position.y = pose_data.y;
    odom_data.pose.pose.position.z = pose_data.z;
    tf2::Quaternion q; q.setRPY(pose_data.roll, pose_data.pitch, pose_data.yaw);
    odom_data.pose.pose.orientation.w = q.w();
    odom_data.pose.pose.orientation.x = q.x();
    odom_data.pose.pose.orientation.y = q.y();
    odom_data.pose.pose.orientation.z = q.z();
}

void transToOdom(const msg_interface::msg::KeyFramePose &pose_data, geometry_msgs::msg::TransformStamped &odom_data){
    odom_data.transform.translation.x = pose_data.x;
    odom_data.transform.translation.y = pose_data.y;
    odom_data.transform.translation.z = pose_data.z;
    tf2::Quaternion q; q.setRPY(pose_data.roll, pose_data.pitch, pose_data.yaw);
    odom_data.transform.rotation.w = q.w();
    odom_data.transform.rotation.x = q.x();
    odom_data.transform.rotation.y = q.y();
    odom_data.transform.rotation.z = q.z();
}

void transToPointPose(const msg_interface::msg::KeyFramePose &pose_data, PointTypePose &point_pose){
    point_pose.x = pose_data.x;
    point_pose.y = pose_data.y;
    point_pose.z = pose_data.z;
    point_pose.roll = pose_data.roll;
    point_pose.pitch = pose_data.pitch;
    point_pose.yaw = pose_data.yaw;
    point_pose.time = pose_data.time;
    point_pose.intensity = pose_data.index;
}
void transToArray(const Eigen::Matrix4f &transform, float* data){
    double roll, pitch, yaw; 
    Eigen::Quaternionf q_eigen(transform.block<3,3>(0,0));
    tf2::Quaternion q(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w()); 
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    data[0] = (float)roll;
    data[1] = (float)pitch;
    data[2] = (float)yaw;
    data[3] = transform(0 , 3);
    data[4] = transform(1 , 3);
    data[5] = transform(2 , 3);
}

}
