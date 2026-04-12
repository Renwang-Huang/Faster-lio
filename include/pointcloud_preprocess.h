#ifndef FASTER_LIO_POINTCLOUD_PROCESSING_H
#define FASTER_LIO_POINTCLOUD_PROCESSING_H

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
    
using LivoxMsgConstPtr = livox_ros_driver2::msg::CustomMsg::ConstSharedPtr;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdint>

#include "common_lib.h"

namespace livox_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint8_t tag;
    uint8_t line;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace livox_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::Point,
                                (float, x, x)
                                (float, y, y)
                                (float, z, z)
                                (float, intensity, intensity)
                                (std::uint8_t, tag, tag)
                                (std::uint8_t, line, line)
                                (double, timestamp, timestamp)
)
// clang-format on

namespace faster_lio {

enum class LidarType { AVIA = 1, LIVOX };

class PointCloudPreprocess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudPreprocess() = default;
    ~PointCloudPreprocess() = default;

    /// processors
    void Process(const LivoxMsgConstPtr &msg, PointCloudType::Ptr &pcl_out);
    void Process(const PointCloud2ConstPtr &msg, PointCloudType::Ptr &pcl_out);
    void Set(LidarType lid_type, double bld, int pfilt_num);

    // accessors
    double &Blind() { return blind_; }
    int &NumScans() { return num_scans_; }
    int &PointFilterNum() { return point_filter_num_; }
    bool &FeatureEnabled() { return feature_enabled_; }
    float &TimeScale() { return time_scale_; }
    LidarType GetLidarType() const { return lidar_type_; }
    void SetLidarType(LidarType lt) { lidar_type_ = lt; }

   private:
    void AviaHandler(const LivoxMsgConstPtr &msg);
    void LivoxHandler(const PointCloud2ConstPtr &msg);

    PointCloudType cloud_full_, cloud_out_;

    LidarType lidar_type_ = LidarType::AVIA;
    bool feature_enabled_ = false;
    int point_filter_num_ = 1;
    int num_scans_ = 6;
    double blind_ = 0.01;
    float time_scale_ = 1e-3;
    bool given_offset_time_ = false;
};
}  // namespace faster_lio

#endif