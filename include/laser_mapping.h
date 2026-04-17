#ifndef FASTER_LIO_LASER_MAPPING_H
#define FASTER_LIO_LASER_MAPPING_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <thread>
#include <deque>
#include <mutex>

#include "imu_processing.hpp"
#include "ivox3d/ivox3d.h"
#include "options.h"
#include "pointcloud_preprocess.h"

#include <livox_ros_driver2/msg/custom_msg.hpp>

namespace faster_lio {

class LaserMapping {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;

    LaserMapping();

    ~LaserMapping() {
        scan_down_body_ = nullptr;
        scan_undistort_ = nullptr;
        scan_down_world_ = nullptr;
        LOG(INFO) << "laser mapping deconstruct";
    }
    
    void Run();
    bool InitROS(rclcpp::Node::SharedPtr node);

    void LivoxPCLCallBack(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg);
    void IMUCallBack(const sensor_msgs::msg::Imu::ConstSharedPtr msg_raw);
    bool SyncPackages();

    /// interface of mtk, customized obseravtion model
    void ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);

    ////////////////////////////// debug save / show //////////////////////////////
    void PublishPath(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path);
    void PublishOdometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pub_odom_aft_mapped);
    void PublishFrameWorld();
    void PublishFrameBody(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub_laser_cloud_body);
    void PublishFrameEffectWorld(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub_laser_cloud_effect_world);

   private:
    template <typename T>
    void SetPosestamp(T &out);

    bool LoadParams(rclcpp::Node::SharedPtr node);
    void SubAndPubToROS(rclcpp::Node::SharedPtr node);
    void MapIncremental();

    void PointBodyToWorld(PointType const *pi, PointType *const po);
    void PointBodyToWorld(const common::V3F &pi, PointType *const po);
    void PointBodyLidarToIMU(PointType const *const pi, PointType *const po);

   private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;                 // imu process

    /// local map related
    float det_range_ = 300.0f;
    double cube_len_ = 0;
    double filter_size_map_min_ = 0;
    bool localmap_initialized_ = false;

    std::vector<double> extrinT_{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // lidar-imu rotation
    std::string map_file_path_;

    /// point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};   // scan after undistortion
    CloudPtr scan_down_body_{new PointCloudType()};   // downsampled scan in body
    CloudPtr scan_down_world_{new PointCloudType()};  // downsampled scan in world
    std::vector<PointVector> nearest_points_;         // nearest points of current scan
    common::VV4F corr_pts_;                           // inlier pts
    common::VV4F corr_norm_;                          // inlier plane norms
    pcl::VoxelGrid<PointType> voxel_scan_;            // voxel filter for current scan
    std::vector<float> residuals_;                    // point-to-plane residuals
    std::vector<char> point_selected_surf_;           // selected points
    common::VV4F plane_coef_;                         // plane coeffs

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_std_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_world_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_effect_world_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_aft_mapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    
    std::string tf_imu_frame_;
    std::string tf_world_frame_;

    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;
    std::deque<PointCloudType::Ptr> lidar_buffer_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;
    
    nav_msgs::msg::Odometry odom_aft_mapped_;
    
    bool lidar_pushed_ = false;
    bool time_sync_en_ = false;
    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = -1.0;
    double lidar_end_time_ = 0.0;
    double last_timestamp_imu_ = -1.0;
    double first_lidar_time_ = 0.0;

    // statistics and flags
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    int pcd_index_ = 0;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    bool timediff_set_flg_ = false;
    int effect_feat_num_ = 0;

    /////////////////////////  EKF inputs and output /////////////////////////
    common::MeasureGroup measures_;                    // sync IMU and lidar scan
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;  // esekf
    state_ikfom state_point_;                          // ekf current state
    vect3 pos_lidar_;                                  // lidar position after eskf update
    common::V3D euler_cur_ = common::V3D::Zero();      // rotation in euler angles
    bool extrinsic_est_en_ = true;

    /////////////////////////  debug show / save /////////////////////////
    bool run_in_offline_ = false;
    bool path_pub_en_ = true;
    bool scan_pub_en_ = false;
    bool dense_pub_en_ = false;
    bool scan_body_pub_en_ = false;
    bool scan_effect_pub_en_ = false;
    bool pcd_save_en_ = false;
    bool runtime_pos_log_ = true;
    int pcd_save_interval_ = -1;
    std::string dataset_;

    PointCloudType::Ptr pcl_wait_save_{new PointCloudType()}; 
    nav_msgs::msg::Path path_;
    geometry_msgs::msg::PoseStamped msg_body_pose_;
};
}  // namespace faster_lio

#endif  // FASTER_LIO_LASER_MAPPING_H
