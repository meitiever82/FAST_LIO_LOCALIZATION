// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <omp.h>
#include <mutex>
#include <cmath>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <Eigen/Core>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// PCL headers
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

// Livox driver
#include <livox_ros_driver2/msg/custom_msg.hpp>

// Local headers
#include "IMU_Processing.hpp"
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

#define TF_MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8],v[9],v[10],v[11],v[12],v[13],v[14],v[15]

using namespace std::chrono_literals;

// Forward declaration for global callback wrapper
class FastLIONode;
FastLIONode* g_fastlio_node = nullptr;

// Forward declaration of the wrapper function (defined after the class)
void h_share_model_wrapper(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);

class FastLIONode : public rclcpp::Node
{
public:
    FastLIONode() : Node("laserMapping")
    {
        // Set global pointer for callback wrapper
        g_fastlio_node = this;

        // Initialize parameters
        initParameters();

        // Initialize variables
        initVariables();

        // Create publishers
        createPublishers();

        // Create subscribers
        createSubscribers();

        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Create timer for main loop
        timer_ = this->create_wall_timer(
            200us, std::bind(&FastLIONode::mainLoop, this));

        RCLCPP_INFO(this->get_logger(), "FastLIO Node initialized");
    }

private:
    void initParameters()
    {
        // Declare and get parameters
        this->declare_parameter("publish.path_en", true);
        this->declare_parameter("publish.scan_publish_en", true);
        this->declare_parameter("publish.dense_publish_en", true);
        this->declare_parameter("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter("max_iteration", 4);
        this->declare_parameter("map_file_path", "");
        this->declare_parameter("common.lid_topic", "/livox/lidar");
        this->declare_parameter("common.imu_topic", "/livox/imu");
        this->declare_parameter("common.path_pcd", "/path");
        this->declare_parameter("common.update_tree_frame", 80);
        this->declare_parameter("common.time_sync_en", false);
        this->declare_parameter("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter("common.approximate_init_pose", std::vector<double>(16, 0.0));
        this->declare_parameter("filter_size_corner", 0.5);
        this->declare_parameter("filter_size_surf", 0.5);
        this->declare_parameter("filter_size_map", 0.5);
        this->declare_parameter("cube_side_length", 200.0);
        this->declare_parameter("mapping.det_range", 300.0);
        this->declare_parameter("mapping.fov_degree", 180.0);
        this->declare_parameter("mapping.gyr_cov", 0.1);
        this->declare_parameter("mapping.acc_cov", 0.1);
        this->declare_parameter("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter("mapping.b_acc_cov", 0.0001);
        this->declare_parameter("preprocess.blind", 0.01);
        this->declare_parameter("preprocess.lidar_type", 1);
        this->declare_parameter("preprocess.scan_line", 16);
        this->declare_parameter("preprocess.timestamp_unit", 2);
        this->declare_parameter("preprocess.scan_rate", 10);
        this->declare_parameter("point_filter_num", 2);
        this->declare_parameter("feature_extract_enable", false);
        this->declare_parameter("runtime_pos_log_enable", false);
        this->declare_parameter("mapping.extrinsic_est_en", true);
        this->declare_parameter("pcd_save.pcd_save_en", false);
        this->declare_parameter("pcd_save.interval", -1);
        this->declare_parameter("mapping.extrinsic_T", std::vector<double>(3, 0.0));
        this->declare_parameter("mapping.extrinsic_R", std::vector<double>(9, 0.0));
        // TF frame names for extrinsic lookup
        this->declare_parameter("mapping.imu_frame", "imu_link");
        this->declare_parameter("mapping.lidar_frame", "rslidar");
        this->declare_parameter("mapping.use_tf_extrinsic", true);  // Try TF first, fallback to config

        path_en_ = this->get_parameter("publish.path_en").as_bool();
        scan_pub_en_ = this->get_parameter("publish.scan_publish_en").as_bool();
        dense_pub_en_ = this->get_parameter("publish.dense_publish_en").as_bool();
        scan_body_pub_en_ = this->get_parameter("publish.scan_bodyframe_pub_en").as_bool();
        NUM_MAX_ITERATIONS_ = this->get_parameter("max_iteration").as_int();
        map_file_path_ = this->get_parameter("map_file_path").as_string();
        lid_topic_ = this->get_parameter("common.lid_topic").as_string();
        imu_topic_ = this->get_parameter("common.imu_topic").as_string();
        map_path_ = this->get_parameter("common.path_pcd").as_string();
        update_thr_ = this->get_parameter("common.update_tree_frame").as_int();
        time_sync_en_ = this->get_parameter("common.time_sync_en").as_bool();
        time_diff_lidar_to_imu_ = this->get_parameter("common.time_offset_lidar_to_imu").as_double();
        init_p_ = this->get_parameter("common.approximate_init_pose").as_double_array();
        filter_size_corner_min_ = this->get_parameter("filter_size_corner").as_double();
        filter_size_surf_min_ = this->get_parameter("filter_size_surf").as_double();
        filter_size_map_min_ = this->get_parameter("filter_size_map").as_double();
        cube_len_ = this->get_parameter("cube_side_length").as_double();
        DET_RANGE_ = this->get_parameter("mapping.det_range").as_double();
        fov_deg_ = this->get_parameter("mapping.fov_degree").as_double();
        gyr_cov_ = this->get_parameter("mapping.gyr_cov").as_double();
        acc_cov_ = this->get_parameter("mapping.acc_cov").as_double();
        b_gyr_cov_ = this->get_parameter("mapping.b_gyr_cov").as_double();
        b_acc_cov_ = this->get_parameter("mapping.b_acc_cov").as_double();

        p_pre_.reset(new Preprocess());
        p_pre_->blind = this->get_parameter("preprocess.blind").as_double();
        p_pre_->lidar_type = this->get_parameter("preprocess.lidar_type").as_int();
        p_pre_->N_SCANS = this->get_parameter("preprocess.scan_line").as_int();
        p_pre_->time_unit = this->get_parameter("preprocess.timestamp_unit").as_int();
        p_pre_->SCAN_RATE = this->get_parameter("preprocess.scan_rate").as_int();
        p_pre_->point_filter_num = this->get_parameter("point_filter_num").as_int();
        p_pre_->feature_enabled = this->get_parameter("feature_extract_enable").as_bool();

        runtime_pos_log_ = this->get_parameter("runtime_pos_log_enable").as_bool();
        extrinsic_est_en_ = this->get_parameter("mapping.extrinsic_est_en").as_bool();
        pcd_save_en_ = this->get_parameter("pcd_save.pcd_save_en").as_bool();
        pcd_save_interval_ = this->get_parameter("pcd_save.interval").as_int();
        extrinT_ = this->get_parameter("mapping.extrinsic_T").as_double_array();
        extrinR_ = this->get_parameter("mapping.extrinsic_R").as_double_array();
        imu_frame_ = this->get_parameter("mapping.imu_frame").as_string();
        lidar_frame_ = this->get_parameter("mapping.lidar_frame").as_string();
        use_tf_extrinsic_ = this->get_parameter("mapping.use_tf_extrinsic").as_bool();

        RCLCPP_INFO(this->get_logger(), "LiDAR type: %d", p_pre_->lidar_type);
        RCLCPP_INFO(this->get_logger(), "IMU frame: %s, LiDAR frame: %s", imu_frame_.c_str(), lidar_frame_.c_str());
    }

    void initVariables()
    {
        // Initialize point clouds
        featsFromMap_.reset(new PointCloudXYZI());
        feats_undistort_.reset(new PointCloudXYZI());
        feats_down_body_.reset(new PointCloudXYZI());
        feats_down_world_.reset(new PointCloudXYZI());
        initial_map_.reset(new PointCloudXYZI());
        normvec_.reset(new PointCloudXYZI(100000, 1));
        laserCloudOri_.reset(new PointCloudXYZI(100000, 1));
        corr_normvect_.reset(new PointCloudXYZI(100000, 1));
        _featsArray_.reset(new PointCloudXYZI());
        pcl_wait_pub_.reset(new PointCloudXYZI(500000, 1));
        pcl_wait_save_.reset(new PointCloudXYZI());

        // Initialize path
        path_.header.stamp = this->now();
        path_.header.frame_id = "camera_init";

        // Initialize filters
        FOV_DEG_ = (fov_deg_ + 10.0) > 179.9 ? 179.9 : (fov_deg_ + 10.0);
        HALF_FOV_COS_ = cos((FOV_DEG_) * 0.5 * PI_M / 180.0);

        memset(point_selected_surf_, true, sizeof(point_selected_surf_));
        memset(res_last_, -1000.0f, sizeof(res_last_));
        downSizeFilterSurf_.setLeafSize(filter_size_surf_min_, filter_size_surf_min_, filter_size_surf_min_);
        downSizeFilterMap_.setLeafSize(filter_size_map_min_, filter_size_map_min_, filter_size_map_min_);

        // Initialize TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize extrinsics - try TF first, fallback to config
        bool tf_extrinsic_ok = false;
        if (use_tf_extrinsic_) {
            tf_extrinsic_ok = getExtrinsicFromTF();
        }

        if (!tf_extrinsic_ok) {
            // Use config file values
            RCLCPP_INFO(this->get_logger(), "Using extrinsic from config file");
            Lidar_T_wrt_IMU_ << VEC_FROM_ARRAY(extrinT_);
            Lidar_R_wrt_IMU_ << MAT_FROM_ARRAY(extrinR_);
        }

        RCLCPP_INFO(this->get_logger(), "Extrinsic T (LiDAR wrt IMU): [%.4f, %.4f, %.4f]",
            Lidar_T_wrt_IMU_(0), Lidar_T_wrt_IMU_(1), Lidar_T_wrt_IMU_(2));
        RCLCPP_INFO(this->get_logger(), "Extrinsic R (LiDAR wrt IMU):\n[%.4f, %.4f, %.4f]\n[%.4f, %.4f, %.4f]\n[%.4f, %.4f, %.4f]",
            Lidar_R_wrt_IMU_(0,0), Lidar_R_wrt_IMU_(0,1), Lidar_R_wrt_IMU_(0,2),
            Lidar_R_wrt_IMU_(1,0), Lidar_R_wrt_IMU_(1,1), Lidar_R_wrt_IMU_(1,2),
            Lidar_R_wrt_IMU_(2,0), Lidar_R_wrt_IMU_(2,1), Lidar_R_wrt_IMU_(2,2));

        if (init_p_.size() >= 16) {
            init_pose_ << TF_MAT_FROM_ARRAY(init_p_);
        } else {
            init_pose_.setIdentity();
        }

        // Initialize IMU processor
        p_imu_.reset(new ImuProcess());
        p_imu_->set_extrinsic(Lidar_T_wrt_IMU_, Lidar_R_wrt_IMU_);
        p_imu_->set_gyr_cov(V3D(gyr_cov_, gyr_cov_, gyr_cov_));
        p_imu_->set_acc_cov(V3D(acc_cov_, acc_cov_, acc_cov_));
        p_imu_->set_gyr_bias_cov(V3D(b_gyr_cov_, b_gyr_cov_, b_gyr_cov_));
        p_imu_->set_acc_bias_cov(V3D(b_acc_cov_, b_acc_cov_, b_acc_cov_));

        // Initialize EKF
        double epsi[23] = {0.001};
        std::fill(epsi, epsi+23, 0.001);
        kf_.init_dyn_share(get_f, df_dx, df_dw,
            h_share_model_wrapper,
            NUM_MAX_ITERATIONS_, epsi);

        // Open log files if needed
        if (runtime_pos_log_) {
            std::string pos_log_dir = std::string(ROOT_DIR) + "/Log/pos_log.txt";
            fp_ = fopen(pos_log_dir.c_str(), "w");
            fout_pre_.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
            fout_out_.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
        }
    }

    bool getExtrinsicFromTF()
    {
        // Try to get extrinsic from TF (imu_frame -> lidar_frame)
        // This gives us the transform of LiDAR in IMU frame
        RCLCPP_INFO(this->get_logger(), "Trying to get extrinsic from TF: %s -> %s",
            imu_frame_.c_str(), lidar_frame_.c_str());

        // Wait for TF to be available (max 3 seconds)
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            // Wait for transform with timeout
            if (!tf_buffer_->canTransform(imu_frame_, lidar_frame_, tf2::TimePointZero,
                                          tf2::durationFromSec(3.0))) {
                RCLCPP_WARN(this->get_logger(), "TF transform not available: %s -> %s",
                    imu_frame_.c_str(), lidar_frame_.c_str());
                return false;
            }

            transform_stamped = tf_buffer_->lookupTransform(
                imu_frame_, lidar_frame_, tf2::TimePointZero);

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get TF transform: %s", ex.what());
            return false;
        }

        // Extract translation
        Lidar_T_wrt_IMU_(0) = transform_stamped.transform.translation.x;
        Lidar_T_wrt_IMU_(1) = transform_stamped.transform.translation.y;
        Lidar_T_wrt_IMU_(2) = transform_stamped.transform.translation.z;

        // Extract rotation (quaternion to rotation matrix)
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
        tf2::Matrix3x3 tf_rot(q);

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                Lidar_R_wrt_IMU_(i, j) = tf_rot[i][j];
            }
        }

        RCLCPP_INFO(this->get_logger(), "Successfully got extrinsic from TF");
        return true;
    }

    void createPublishers()
    {
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100000);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 100000);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100000);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100000);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100000);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 100000);
    }

    void createSubscribers()
    {
        auto qos = rclcpp::SensorDataQoS();

        if (p_pre_->lidar_type == AVIA) {
            sub_pcl_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                lid_topic_, qos,
                std::bind(&FastLIONode::livox_pcl_cbk, this, std::placeholders::_1));
        } else {
            sub_pcl_standard_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic_, qos,
                std::bind(&FastLIONode::standard_pcl_cbk, this, std::placeholders::_1));
        }

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 200,
            std::bind(&FastLIONode::imu_cbk, this, std::placeholders::_1));
    }

    void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_buffer_);
        scan_count_++;
        double preprocess_start_time = omp_get_wtime();
        double timestamp = rclcpp::Time(msg->header.stamp).seconds();

        if (timestamp < last_timestamp_lidar_) {
            RCLCPP_ERROR(this->get_logger(), "lidar loop back, clear buffer");
            lidar_buffer_.clear();
        }

        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        p_pre_->process(msg, ptr);
        lidar_buffer_.push_back(ptr);
        time_buffer_.push_back(timestamp);
        last_timestamp_lidar_ = timestamp;
        s_plot11_[scan_count_] = omp_get_wtime() - preprocess_start_time;
    }

    void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_buffer_);
        double preprocess_start_time = omp_get_wtime();
        scan_count_++;
        double timestamp = rclcpp::Time(msg->header.stamp).seconds();

        if (timestamp < last_timestamp_lidar_) {
            RCLCPP_ERROR(this->get_logger(), "lidar loop back, clear buffer");
            lidar_buffer_.clear();
        }
        last_timestamp_lidar_ = timestamp;

        if (!time_sync_en_ && std::abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0
            && !imu_buffer_.empty() && !lidar_buffer_.empty()) {
            RCLCPP_WARN(this->get_logger(), "IMU and LiDAR not Synced, IMU time: %f, lidar header time: %f",
                last_timestamp_imu_, last_timestamp_lidar_);
        }

        if (time_sync_en_ && !timediff_set_flg_ && std::abs(last_timestamp_lidar_ - last_timestamp_imu_) > 1
            && !imu_buffer_.empty()) {
            timediff_set_flg_ = true;
            timediff_lidar_wrt_imu_ = last_timestamp_lidar_ + 0.1 - last_timestamp_imu_;
            RCLCPP_INFO(this->get_logger(), "Self sync IMU and LiDAR, time diff is %.10lf", timediff_lidar_wrt_imu_);
        }

        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        p_pre_->process(msg, ptr);
        lidar_buffer_.push_back(ptr);
        time_buffer_.push_back(last_timestamp_lidar_);

        s_plot11_[scan_count_] = omp_get_wtime() - preprocess_start_time;
    }

    void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg_in)
    {
        publish_count_++;
        sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

        double timestamp = rclcpp::Time(msg_in->header.stamp).seconds() - time_diff_lidar_to_imu_;
        if (std::abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_) {
            timestamp = timediff_lidar_wrt_imu_ + rclcpp::Time(msg_in->header.stamp).seconds();
        }
        msg->header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));

        std::lock_guard<std::mutex> lock(mtx_buffer_);

        if (timestamp < last_timestamp_imu_) {
            RCLCPP_WARN(this->get_logger(), "imu loop back, clear buffer");
            imu_buffer_.clear();
        }

        last_timestamp_imu_ = timestamp;
        imu_buffer_.push_back(msg);
    }

    bool sync_packages(MeasureGroup &meas)
    {
        if (lidar_buffer_.empty() || imu_buffer_.empty()) {
            return false;
        }

        if (!lidar_pushed_) {
            meas.lidar = lidar_buffer_.front();
            meas.lidar_beg_time = time_buffer_.front();
            if (meas.lidar->points.size() <= 1) {
                lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
                RCLCPP_WARN(this->get_logger(), "Too few input point cloud!");
            } else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_) {
                lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
            } else {
                scan_num_++;
                lidar_end_time_ = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
                lidar_mean_scantime_ += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
            }

            meas.lidar_end_time = lidar_end_time_;
            lidar_pushed_ = true;
        }

        if (last_timestamp_imu_ < lidar_end_time_) {
            return false;
        }

        double imu_time = rclcpp::Time(imu_buffer_.front()->header.stamp).seconds();
        meas.imu.clear();
        while (!imu_buffer_.empty() && (imu_time < lidar_end_time_)) {
            imu_time = rclcpp::Time(imu_buffer_.front()->header.stamp).seconds();
            if (imu_time > lidar_end_time_) break;
            meas.imu.push_back(imu_buffer_.front());
            imu_buffer_.pop_front();
        }

        lidar_buffer_.pop_front();
        time_buffer_.pop_front();
        lidar_pushed_ = false;
        return true;
    }

    void pointBodyToWorld(PointType const * const pi, PointType * const po)
    {
        V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(state_point_.rot * (state_point_.offset_R_L_I*p_body + state_point_.offset_T_L_I) + state_point_.pos);

        po->x = p_global(0);
        po->y = p_global(1);
        po->z = p_global(2);
        po->intensity = pi->intensity;
    }

    template<typename T>
    void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po)
    {
        V3D p_body(pi[0], pi[1], pi[2]);
        V3D p_global(state_point_.rot * (state_point_.offset_R_L_I*p_body + state_point_.offset_T_L_I) + state_point_.pos);

        po[0] = p_global(0);
        po[1] = p_global(1);
        po[2] = p_global(2);
    }

    void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
    {
        V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(state_point_.rot * (state_point_.offset_R_L_I*p_body + state_point_.offset_T_L_I) + state_point_.pos);

        po->x = p_global(0);
        po->y = p_global(1);
        po->z = p_global(2);
        po->intensity = pi->intensity;
    }

    void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
    {
        V3D p_body_lidar(pi->x, pi->y, pi->z);
        V3D p_body_imu(state_point_.offset_R_L_I*p_body_lidar + state_point_.offset_T_L_I);

        po->x = p_body_imu(0);
        po->y = p_body_imu(1);
        po->z = p_body_imu(2);
        po->intensity = pi->intensity;
    }

    void lasermap_fov_segment()
    {
        cub_needrm_.clear();
        kdtree_delete_counter_ = 0;
        kdtree_delete_time_ = 0.0;
        pointBodyToWorld(XAxisPoint_body_, XAxisPoint_world_);
        V3D pos_LiD = pos_lid_;

        if (!Localmap_Initialized_) {
            for (int i = 0; i < 3; i++) {
                LocalMap_Points_.vertex_min[i] = pos_LiD(i) - cube_len_ / 2.0;
                LocalMap_Points_.vertex_max[i] = pos_LiD(i) + cube_len_ / 2.0;
            }
            Localmap_Initialized_ = true;
            return;
        }

        float dist_to_map_edge[3][2];
        bool need_move = false;
        for (int i = 0; i < 3; i++) {
            dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points_.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points_.vertex_max[i]);
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD_ * DET_RANGE_ ||
                dist_to_map_edge[i][1] <= MOV_THRESHOLD_ * DET_RANGE_) need_move = true;
        }
        if (!need_move) return;

        BoxPointType New_LocalMap_Points, tmp_boxpoints;
        New_LocalMap_Points = LocalMap_Points_;
        float mov_dist = std::max((cube_len_ - 2.0 * MOV_THRESHOLD_ * DET_RANGE_) * 0.5 * 0.9,
                                   double(DET_RANGE_ * (MOV_THRESHOLD_ - 1)));
        for (int i = 0; i < 3; i++) {
            tmp_boxpoints = LocalMap_Points_;
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD_ * DET_RANGE_) {
                New_LocalMap_Points.vertex_max[i] -= mov_dist;
                New_LocalMap_Points.vertex_min[i] -= mov_dist;
                tmp_boxpoints.vertex_min[i] = LocalMap_Points_.vertex_max[i] - mov_dist;
                cub_needrm_.push_back(tmp_boxpoints);
            } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD_ * DET_RANGE_) {
                New_LocalMap_Points.vertex_max[i] += mov_dist;
                New_LocalMap_Points.vertex_min[i] += mov_dist;
                tmp_boxpoints.vertex_max[i] = LocalMap_Points_.vertex_min[i] + mov_dist;
                cub_needrm_.push_back(tmp_boxpoints);
            }
        }
        LocalMap_Points_ = New_LocalMap_Points;

        PointVector points_history;
        ikdtree_.acquire_removed_points(points_history);
        double delete_begin = omp_get_wtime();
        if (cub_needrm_.size() > 0) kdtree_delete_counter_ = ikdtree_.Delete_Point_Boxes(cub_needrm_);
        kdtree_delete_time_ = omp_get_wtime() - delete_begin;
    }

    void map_incremental()
    {
        PointVector PointToAdd;
        PointVector PointNoNeedDownsample;
        PointToAdd.reserve(feats_down_size_);
        PointNoNeedDownsample.reserve(feats_down_size_);

        for (int i = 0; i < feats_down_size_; i++) {
            pointBodyToWorld(&(feats_down_body_->points[i]), &(feats_down_world_->points[i]));
            if (!Nearest_Points_[i].empty() && flg_EKF_inited_) {
                const PointVector &points_near = Nearest_Points_[i];
                bool need_add = true;
                BoxPointType Box_of_Point;
                PointType downsample_result, mid_point;
                mid_point.x = floor(feats_down_world_->points[i].x/filter_size_map_min_)*filter_size_map_min_ + 0.5 * filter_size_map_min_;
                mid_point.y = floor(feats_down_world_->points[i].y/filter_size_map_min_)*filter_size_map_min_ + 0.5 * filter_size_map_min_;
                mid_point.z = floor(feats_down_world_->points[i].z/filter_size_map_min_)*filter_size_map_min_ + 0.5 * filter_size_map_min_;
                float dist = calc_dist(feats_down_world_->points[i], mid_point);
                if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min_ &&
                    fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min_ &&
                    fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min_) {
                    PointNoNeedDownsample.push_back(feats_down_world_->points[i]);
                    continue;
                }
                for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                    if (points_near.size() < NUM_MATCH_POINTS) break;
                    if (calc_dist(points_near[readd_i], mid_point) < dist) {
                        need_add = false;
                        break;
                    }
                }
                if (need_add) PointToAdd.push_back(feats_down_world_->points[i]);
            } else {
                PointToAdd.push_back(feats_down_world_->points[i]);
            }
        }

        double st_time = omp_get_wtime();
        if (scan_counter_ > update_thr_ && update_thr_ != -1) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Started Updating Map tree.");
            add_point_size_ = ikdtree_.Add_Points(PointToAdd, true);
            ikdtree_.Add_Points(PointNoNeedDownsample, false);
            add_point_size_ = PointToAdd.size() + PointNoNeedDownsample.size();
            kdtree_incremental_time_ = omp_get_wtime() - st_time;
        }
    }

public:
    // Public method needed for global wrapper callback
    void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
    {
        double match_start = omp_get_wtime();
        laserCloudOri_->clear();
        corr_normvect_->clear();
        total_residual_ = 0.0;

#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
#endif
        for (int i = 0; i < feats_down_size_; i++) {
            PointType &point_body = feats_down_body_->points[i];
            PointType &point_world = feats_down_world_->points[i];

            V3D p_body(point_body.x, point_body.y, point_body.z);
            V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
            point_world.x = p_global(0);
            point_world.y = p_global(1);
            point_world.z = p_global(2);
            point_world.intensity = point_body.intensity;

            std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
            auto &points_near = Nearest_Points_[i];

            if (ekfom_data.converge) {
                ikdtree_.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                point_selected_surf_[i] = points_near.size() < NUM_MATCH_POINTS ? false :
                    pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
            }

            if (!point_selected_surf_[i]) continue;

            VF(4) pabcd;
            point_selected_surf_[i] = false;
            if (esti_plane(pabcd, points_near, 0.1f)) {
                float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                float ss = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                if (ss > 0.9) {
                    point_selected_surf_[i] = true;
                    normvec_->points[i].x = pabcd(0);
                    normvec_->points[i].y = pabcd(1);
                    normvec_->points[i].z = pabcd(2);
                    normvec_->points[i].intensity = pd2;
                    res_last_[i] = abs(pd2);
                }
            }
        }

        effct_feat_num_ = 0;
        for (int i = 0; i < feats_down_size_; i++) {
            if (point_selected_surf_[i]) {
                laserCloudOri_->points[effct_feat_num_] = feats_down_body_->points[i];
                corr_normvect_->points[effct_feat_num_] = normvec_->points[i];
                total_residual_ += res_last_[i];
                effct_feat_num_++;
            }
        }

        if (effct_feat_num_ < 1) {
            ekfom_data.valid = false;
            RCLCPP_WARN(this->get_logger(), "No Effective Points!");
            return;
        }

        res_mean_last_ = total_residual_ / effct_feat_num_;
        match_time_ += omp_get_wtime() - match_start;
        double solve_start_ = omp_get_wtime();

        ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num_, 12);
        ekfom_data.h.resize(effct_feat_num_);

        for (int i = 0; i < effct_feat_num_; i++) {
            const PointType &laser_p = laserCloudOri_->points[i];
            V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
            M3D point_be_crossmat;
            point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
            V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(point_this);

            const PointType &norm_p = corr_normvect_->points[i];
            V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

            V3D C(s.rot.conjugate() * norm_vec);
            V3D A(point_crossmat * C);
            if (extrinsic_est_en_) {
                V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
            } else {
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            }

            ekfom_data.h(i) = -norm_p.intensity;
        }
        solve_time_ += omp_get_wtime() - solve_start_;
    }

private:
    void publish_frame_world()
    {
        if (scan_pub_en_) {
            PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en_ ? feats_undistort_ : feats_down_body_);
            int size = laserCloudFullRes->points.size();
            PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

            for (int i = 0; i < size; i++) {
                RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
            }

            sensor_msgs::msg::PointCloud2 laserCloudmsg;
            pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
            laserCloudmsg.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time_ * 1e9));
            laserCloudmsg.header.frame_id = "camera_init";
            pubLaserCloudFull_->publish(laserCloudmsg);
            publish_count_ -= PUBFRAME_PERIOD;
        }

        if (pcd_save_en_) {
            int size = feats_undistort_->points.size();
            PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

            for (int i = 0; i < size; i++) {
                RGBpointBodyToWorld(&feats_undistort_->points[i], &laserCloudWorld->points[i]);
            }
            *pcl_wait_save_ += *laserCloudWorld;

            static int scan_wait_num = 0;
            scan_wait_num++;
            if (pcl_wait_save_->size() > 0 && pcd_save_interval_ > 0 && scan_wait_num >= pcd_save_interval_) {
                pcd_index_++;
                std::string all_points_dir(std::string(ROOT_DIR) + "PCD/scans_" + std::to_string(pcd_index_) + ".pcd");
                pcl::PCDWriter pcd_writer;
                RCLCPP_INFO(this->get_logger(), "current scan saved to /PCD/%s", all_points_dir.c_str());
                pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
                pcl_wait_save_->clear();
                scan_wait_num = 0;
            }
        }
    }

    void publish_frame_body()
    {
        int size = feats_undistort_->points.size();
        PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            RGBpointBodyLidarToIMU(&feats_undistort_->points[i], &laserCloudIMUBody->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
        laserCloudmsg.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time_ * 1e9));
        laserCloudmsg.header.frame_id = "base_link";
        pubLaserCloudFull_body_->publish(laserCloudmsg);
        publish_count_ -= PUBFRAME_PERIOD;
    }

    void publish_odometry()
    {
        odomAftMapped_.header.frame_id = "camera_init";
        odomAftMapped_.child_frame_id = "base_link";
        odomAftMapped_.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time_ * 1e9));

        odomAftMapped_.pose.pose.position.x = state_point_.pos(0);
        odomAftMapped_.pose.pose.position.y = state_point_.pos(1);
        odomAftMapped_.pose.pose.position.z = state_point_.pos(2);
        odomAftMapped_.pose.pose.orientation.x = geoQuat_.x;
        odomAftMapped_.pose.pose.orientation.y = geoQuat_.y;
        odomAftMapped_.pose.pose.orientation.z = geoQuat_.z;
        odomAftMapped_.pose.pose.orientation.w = geoQuat_.w;

        if (first_odom_) {
            odomAftMapped_.twist.twist.linear.x = 0.0;
            odomAftMapped_.twist.twist.linear.y = 0.0;
            odomAftMapped_.twist.twist.linear.z = 0.0;
            odomAftMapped_.twist.twist.angular.x = 0.0;
            odomAftMapped_.twist.twist.angular.y = 0.0;
            odomAftMapped_.twist.twist.angular.z = 0.0;
            first_odom_ = false;
        } else {
            double dt = rclcpp::Time(odomAftMapped_.header.stamp).seconds() -
                        rclcpp::Time(prev_odomAftMapped_.header.stamp).seconds();
            odomAftMapped_.twist.twist.linear.x = (odomAftMapped_.pose.pose.position.x - prev_odomAftMapped_.pose.pose.position.x) / dt;
            odomAftMapped_.twist.twist.linear.y = (odomAftMapped_.pose.pose.position.y - prev_odomAftMapped_.pose.pose.position.y) / dt;
            odomAftMapped_.twist.twist.linear.z = (odomAftMapped_.pose.pose.position.z - prev_odomAftMapped_.pose.pose.position.z) / dt;
        }

        pubOdomAftMapped_->publish(odomAftMapped_);

        // Publish TF
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = odomAftMapped_.header.stamp;
        transform.header.frame_id = "camera_init";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = odomAftMapped_.pose.pose.position.x;
        transform.transform.translation.y = odomAftMapped_.pose.pose.position.y;
        transform.transform.translation.z = odomAftMapped_.pose.pose.position.z;
        transform.transform.rotation = odomAftMapped_.pose.pose.orientation;
        tf_broadcaster_->sendTransform(transform);

        prev_odomAftMapped_ = odomAftMapped_;
    }

    void publish_path()
    {
        geometry_msgs::msg::PoseStamped msg_body_pose;
        msg_body_pose.pose.position = odomAftMapped_.pose.pose.position;
        msg_body_pose.pose.orientation = odomAftMapped_.pose.pose.orientation;
        msg_body_pose.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time_ * 1e9));
        msg_body_pose.header.frame_id = "camera_init";

        static int jjj = 0;
        jjj++;
        if (jjj % 10 == 0) {
            path_.poses.push_back(msg_body_pose);
            pubPath_->publish(path_);
        }
    }

    void mainLoop()
    {
        if (flg_exit_) return;

        std::lock_guard<std::mutex> lock(mtx_buffer_);

        if (!system_initialized_) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Initializing Map ...");

            // Try loading as PointXYZINormal first, if fails try PointXYZI and convert
            int load_result = pcl::io::loadPCDFile<pcl::PointXYZINormal>(map_path_, *initial_map_);
            if (load_result == -1) {
                RCLCPP_ERROR(this->get_logger(), "Lidar cloud reading failed.");
                return;
            }

            // Check if normals are all zero (meaning PCD only had XYZI fields)
            bool has_valid_normals = false;
            for (size_t i = 0; i < std::min(initial_map_->points.size(), size_t(100)); ++i) {
                const auto& pt = initial_map_->points[i];
                if (pt.normal_x != 0.0f || pt.normal_y != 0.0f || pt.normal_z != 0.0f) {
                    has_valid_normals = true;
                    break;
                }
            }

            if (!has_valid_normals) {
                RCLCPP_INFO(this->get_logger(), "PCD has no normal data, loading as XYZI format...");
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>());
                if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path_, *cloud_xyzi) == -1) {
                    RCLCPP_ERROR(this->get_logger(), "Lidar cloud reading failed (XYZI format).");
                    return;
                }

                // Convert PointXYZI to PointXYZINormal
                initial_map_->clear();
                initial_map_->reserve(cloud_xyzi->size());
                for (const auto& pt : cloud_xyzi->points) {
                    pcl::PointXYZINormal pt_normal;
                    pt_normal.x = pt.x;
                    pt_normal.y = pt.y;
                    pt_normal.z = pt.z;
                    pt_normal.intensity = pt.intensity;
                    pt_normal.normal_x = 0.0f;
                    pt_normal.normal_y = 0.0f;
                    pt_normal.normal_z = 0.0f;
                    pt_normal.curvature = 0.0f;
                    initial_map_->push_back(pt_normal);
                }
                RCLCPP_INFO(this->get_logger(), "Loaded %zu points from XYZI format PCD", initial_map_->size());
            } else {
                RCLCPP_INFO(this->get_logger(), "Loaded %zu points with normal data", initial_map_->size());
            }

            pcl::UniformSampling<pcl::PointXYZINormal> downsample;
            downsample.setInputCloud(initial_map_);
            downsample.setRadiusSearch(0.01);
            downsample.filter(*initial_map_);

            sensor_msgs::msg::PointCloud2 laserCloudmsg_map;
            pcl::toROSMsg(*initial_map_, laserCloudmsg_map);
            laserCloudmsg_map.header.frame_id = "camera_init";
            laserCloudmsg_map.header.stamp = this->now();

            std::this_thread::sleep_for(2s);
            pubLaserCloudFull_->publish(laserCloudmsg_map);

            ikdtree_.set_downsample_param(filter_size_map_min_);
            feats_down_world_->resize(feats_down_size_);
            ikdtree_.Build(initial_map_->points);

            RCLCPP_INFO(this->get_logger(), "MAP Initialized");
            system_initialized_ = true;
            lidar_buffer_.clear();
            time_buffer_.clear();
            imu_buffer_.clear();
            return;
        }

        if (sync_packages(Measures_) && system_initialized_) {
            if (flg_first_scan_) {
                first_lidar_time_ = Measures_.lidar_beg_time;
                p_imu_->first_lidar_time = first_lidar_time_;
                flg_first_scan_ = false;
                return;
            }

            double t0, t1, t2, t3, t4, t5;
            match_time_ = 0;
            kdtree_search_time_ = 0.0;
            solve_time_ = 0;
            solve_const_H_time_ = 0;
            t0 = omp_get_wtime();

            p_imu_->Process(Measures_, kf_, feats_undistort_, initial_map_, init_pose_, initialization_tf_);
            state_point_ = kf_.get_x();
            pos_lid_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;

            if (feats_undistort_->empty() || (feats_undistort_ == nullptr)) {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!");
                return;
            }

            flg_EKF_inited_ = (Measures_.lidar_beg_time - first_lidar_time_) < INIT_TIME ? false : true;

            lasermap_fov_segment();

            downSizeFilterSurf_.setInputCloud(feats_undistort_);
            downSizeFilterSurf_.filter(*feats_down_body_);
            t1 = omp_get_wtime();
            feats_down_size_ = feats_down_body_->points.size();

            if (feats_down_size_ < 5) {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!");
                return;
            }

            normvec_->resize(feats_down_size_);
            feats_down_world_->resize(feats_down_size_);

            pointSearchInd_surf_.resize(feats_down_size_);
            Nearest_Points_.resize(feats_down_size_);

            t2 = omp_get_wtime();

            if (first_odom_) {
                Eigen::Matrix3d initialization_rot;
                initialization_rot = state_point_.rot.toRotationMatrix() * initialization_tf_.block<3, 3>(0, 0).cast<double>();
                state_point_.rot = initialization_rot;
                state_point_.pos = state_point_.pos + initialization_tf_.block<3, 1>(0, 3).cast<double>();
                kf_.change_x(state_point_);
            }

            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point_ = kf_.get_x();
            euler_cur_ = SO3ToEuler(state_point_.rot);
            pos_lid_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
            geoQuat_.x = state_point_.rot.coeffs()[0];
            geoQuat_.y = state_point_.rot.coeffs()[1];
            geoQuat_.z = state_point_.rot.coeffs()[2];
            geoQuat_.w = state_point_.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            publish_odometry();
            scan_counter_++;

            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();

            if (scan_pub_en_ || pcd_save_en_) publish_frame_world();
            if (path_en_) publish_path();
            if (scan_pub_en_ && scan_body_pub_en_) publish_frame_body();

            if (runtime_pos_log_) {
                frame_num_++;
                RCLCPP_INFO(this->get_logger(), "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave total: %0.6f",
                    t1-t0, t5-t0);
            }
        }
    }

    // Member variables
    // Parameters
    bool path_en_ = true;
    bool scan_pub_en_ = true;
    bool dense_pub_en_ = true;
    bool scan_body_pub_en_ = true;
    int NUM_MAX_ITERATIONS_ = 4;
    std::string map_file_path_;
    std::string lid_topic_;
    std::string imu_topic_;
    std::string map_path_;
    int update_thr_ = 80;
    bool time_sync_en_ = false;
    double time_diff_lidar_to_imu_ = 0.0;
    std::vector<double> init_p_;
    double filter_size_corner_min_ = 0.5;
    double filter_size_surf_min_ = 0.5;
    double filter_size_map_min_ = 0.5;
    double cube_len_ = 200.0;
    double DET_RANGE_ = 300.0;
    double fov_deg_ = 180.0;
    double gyr_cov_ = 0.1;
    double acc_cov_ = 0.1;
    double b_gyr_cov_ = 0.0001;
    double b_acc_cov_ = 0.0001;
    bool runtime_pos_log_ = false;
    bool extrinsic_est_en_ = true;
    bool pcd_save_en_ = false;
    int pcd_save_interval_ = -1;
    std::vector<double> extrinT_;
    std::vector<double> extrinR_;

    // State variables
    double kdtree_incremental_time_ = 0.0;
    double kdtree_search_time_ = 0.0;
    double kdtree_delete_time_ = 0.0;
    double T1_[MAXN], s_plot_[MAXN], s_plot11_[MAXN];
    double match_time_ = 0, solve_time_ = 0, solve_const_H_time_ = 0;
    int kdtree_size_st_ = 0, kdtree_size_end_ = 0, add_point_size_ = 0, kdtree_delete_counter_ = 0;

    float res_last_[100000] = {0.0};
    const float MOV_THRESHOLD_ = 1.5f;

    std::mutex mtx_buffer_;

    double res_mean_last_ = 0.05, total_residual_ = 0.0;
    double last_timestamp_lidar_ = 0, last_timestamp_imu_ = -1.0;
    double FOV_DEG_ = 0, HALF_FOV_COS_ = 0, total_distance_ = 0, lidar_end_time_ = 0, first_lidar_time_ = 0.0;
    int effct_feat_num_ = 0, time_log_counter_ = 0, scan_count_ = 0, publish_count_ = 0;
    int feats_down_size_ = 0, laserCloudValidNum_ = 0, pcd_index_ = 0;
    bool point_selected_surf_[100000] = {0};
    bool lidar_pushed_ = false, flg_first_scan_ = true, flg_exit_ = false, flg_EKF_inited_ = false;
    bool first_odom_ = true;
    bool system_initialized_ = false;
    size_t scan_counter_ = 0;
    Eigen::Matrix4f initialization_tf_;
    int frame_num_ = 0;
    int scan_num_ = 0;
    double lidar_mean_scantime_ = 0.0;
    double timediff_lidar_wrt_imu_ = 0.0;
    bool timediff_set_flg_ = false;

    std::vector<std::vector<int>> pointSearchInd_surf_;
    std::vector<BoxPointType> cub_needrm_;
    std::vector<PointVector> Nearest_Points_;
    std::deque<double> time_buffer_;
    std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;

    PointCloudXYZI::Ptr featsFromMap_;
    PointCloudXYZI::Ptr feats_undistort_;
    PointCloudXYZI::Ptr feats_down_body_;
    PointCloudXYZI::Ptr feats_down_world_;
    PointCloudXYZI::Ptr initial_map_;
    PointCloudXYZI::Ptr normvec_;
    PointCloudXYZI::Ptr laserCloudOri_;
    PointCloudXYZI::Ptr corr_normvect_;
    PointCloudXYZI::Ptr _featsArray_;
    PointCloudXYZI::Ptr pcl_wait_pub_;
    PointCloudXYZI::Ptr pcl_wait_save_;

    pcl::VoxelGrid<PointType> downSizeFilterSurf_;
    pcl::VoxelGrid<PointType> downSizeFilterMap_;

    KD_TREE<PointType> ikdtree_;

    V3F XAxisPoint_body_{LIDAR_SP_LEN, 0.0, 0.0};
    V3F XAxisPoint_world_{LIDAR_SP_LEN, 0.0, 0.0};
    V3D euler_cur_;
    V3D position_last_{Zero3d};
    V3D Lidar_T_wrt_IMU_{Zero3d};
    M3D Lidar_R_wrt_IMU_{Eye3d};
    Eigen::Matrix4d init_pose_;

    MeasureGroup Measures_;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;
    state_ikfom state_point_;
    vect3 pos_lid_;

    nav_msgs::msg::Path path_;
    nav_msgs::msg::Odometry odomAftMapped_, prev_odomAftMapped_;
    geometry_msgs::msg::Quaternion geoQuat_;

    BoxPointType LocalMap_Points_;
    bool Localmap_Initialized_ = false;

    std::shared_ptr<Preprocess> p_pre_;
    std::shared_ptr<ImuProcess> p_imu_;

    // ROS2 components
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_standard_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // TF extrinsic parameters
    std::string imu_frame_;
    std::string lidar_frame_;
    bool use_tf_extrinsic_ = true;
    rclcpp::TimerBase::SharedPtr timer_;

    // Log files
    FILE *fp_ = nullptr;
    std::ofstream fout_pre_, fout_out_;
};

// Global wrapper function for EKF measurement callback
void h_share_model_wrapper(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    if (g_fastlio_node) {
        g_fastlio_node->h_share_model(s, ekfom_data);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastLIONode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
