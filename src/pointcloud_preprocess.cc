#include "pointcloud_preprocess.h"

#include <glog/logging.h>
#include <execution>

namespace faster_lio {

void PointCloudPreprocess::Set(LidarType lid_type, double bld, int pfilt_num) {
    lidar_type_ = lid_type;
    blind_ = bld;
    point_filter_num_ = pfilt_num;
}

void PointCloudPreprocess::Process(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg, PointCloudType::Ptr &pcl_out) {
    AviaHandler(msg);
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, PointCloudType::Ptr &pcl_out) {
    switch (lidar_type_) {
        case LidarType::LIVOX:
            LivoxHandler(msg);
            break;

        default:
            LOG(ERROR) << "Error LiDAR Type";
            break;
    }
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::AviaHandler(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    int plsize = msg->point_num;

    cloud_out_.reserve(plsize);
    cloud_full_.resize(plsize);

    std::vector<char> is_valid_pt(plsize, false);
    std::vector<uint> index(plsize - 1);
    for (uint i = 0; i < plsize - 1; ++i) {
        index[i] = i + 1;  // 从1开始
    }

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
        if ((msg->points[i].line < num_scans_) &&
            ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
            if (i % point_filter_num_ == 0) {
                cloud_full_[i].x = msg->points[i].x;
                cloud_full_[i].y = msg->points[i].y;
                cloud_full_[i].z = msg->points[i].z;
                cloud_full_[i].intensity = msg->points[i].reflectivity;
                cloud_full_[i].curvature = static_cast<float>(msg->points[i].offset_time) / static_cast<float>(1000000);
                // use curvature as time of each laser points, curvature unit: ms
                // unit of offset_time: nanosecond

                if ((abs(cloud_full_[i].x - cloud_full_[i - 1].x) > 1e-7) ||
                    (abs(cloud_full_[i].y - cloud_full_[i - 1].y) > 1e-7) ||
                    (abs(cloud_full_[i].z - cloud_full_[i - 1].z) > 1e-7) &&
                        (cloud_full_[i].x * cloud_full_[i].x + cloud_full_[i].y * cloud_full_[i].y +
                             cloud_full_[i].z * cloud_full_[i].z >
                         (blind_ * blind_))) {
                    is_valid_pt[i] = true;
                }
            }
        }
    });

    for (uint i = 1; i < plsize; i++) {
        if (is_valid_pt[i]) {
            cloud_out_.points.push_back(cloud_full_[i]);
        }
    }
}

void PointCloudPreprocess::LivoxHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<livox_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();

    cloud_out_.reserve(plsize);
    cloud_full_.resize(plsize);

    std::vector<char> is_valid_pt(plsize, false);
    std::vector<uint> index(plsize - 1);
    for (uint i = 0; i < plsize - 1; ++i) {
        index[i] = i + 1;  // 从1开始
    }

    double timebase = pl_orig.points[0].timestamp;

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
        if ((pl_orig.points[i].line < num_scans_) &&
            ((pl_orig.points[i].tag & 0x30) == 0x10 || (pl_orig.points[i].tag & 0x30) == 0x00)) {
            if (i % point_filter_num_ == 0) {
                cloud_full_[i].x = pl_orig.points[i].x;
                cloud_full_[i].y = pl_orig.points[i].y;
                cloud_full_[i].z = pl_orig.points[i].z;
                cloud_full_[i].intensity = pl_orig.points[i].intensity;
                cloud_full_[i].curvature =
                    static_cast<float>(pl_orig.points[i].timestamp - timebase) / static_cast<float>(1000000);
                // use curvature as time of each laser points, curvature unit: ms
                // unit of offset_time: nanosecond

                if ((abs(cloud_full_[i].x - cloud_full_[i - 1].x) > 1e-7) ||
                    (abs(cloud_full_[i].y - cloud_full_[i - 1].y) > 1e-7) ||
                    (abs(cloud_full_[i].z - cloud_full_[i - 1].z) > 1e-7) &&
                        (cloud_full_[i].x * cloud_full_[i].x + cloud_full_[i].y * cloud_full_[i].y +
                             cloud_full_[i].z * cloud_full_[i].z >
                         (blind_ * blind_))) {
                    is_valid_pt[i] = true;
                }
            }
        }
    });

    for (uint i = 1; i < plsize; i++) {
        if (is_valid_pt[i]) {
            cloud_out_.points.push_back(cloud_full_[i]);
        }
    }
}
}  // namespace faster_lio
