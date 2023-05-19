/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-02-11 17:12:15
 */
#ifndef LOCALIZATION_MAPPING_SENSOR_DATA_COSTOM_POINT_TYPE_
#define LOCALIZATION_MAPPING_SENSOR_DATA_COSTOM_POINT_TYPE_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>


// 不同雷达点云格式定义
enum class SensorType {VELODYNE, OUSTER, LIVOX , Rs};

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (float, time, time)
)

struct RsPointXYZIRT    // 速腾
{
    PCL_ADD_POINT4D
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (RsPointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp)
)


struct OusterPointXYZIRT {
    PCL_ADD_POINT4D
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, noise, noise)
    (std::uint32_t, range, range)
)

struct PointXYZIRPYT{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
}EIGEN_ALIGN16;                         // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
    (float, x, x) (float, y, y)
    (float, z, z) (float, intensity, intensity)
    (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
    (double, time, time)
)

using PointType = pcl::PointXYZI;
using PointXYZIRT = RsPointXYZIRT;   
using PointTypePose = PointXYZIRPYT;

#endif