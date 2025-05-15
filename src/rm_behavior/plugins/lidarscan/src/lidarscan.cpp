// Copyright 2025 Jquark
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "lidarscan/lidarscan.hpp"
#include <algorithm>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <cmath>

namespace lidarscan {

LidarscanNode::LidarscanNode(const rclcpp::NodeOptions & options)
: Node("lidarscan_node", options),
  current_state_(ScanState::IDLE),
  obstacle_scan_index_(0)
{
  // 参数初始化
  this->declare_parameter<std::string>("map_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("gimbal_frame", "gimbal_link");
  this->declare_parameter<double>("scan_speed", 0.3);
  this->declare_parameter<double>("pitch_scan_speed", 0.25);
  this->declare_parameter<double>("min_obstacle_size", 0.1);
  this->declare_parameter<std::string>("terrainmap_topic", "/terrain_map");
  this->declare_parameter<std::string>("gimbal_cmd_topic", "/lidarscan");
  this->declare_parameter<double>("voxel_leaf_size", 0.05);
  this->declare_parameter<double>("smoothing_factor", 0.15, rcl_interfaces::msg::ParameterDescriptor()
                                  .set__description("Smoothing factor for gimbal movement (0 to 1). Smaller is smoother.")
                                  .set__floating_point_range({rcl_interfaces::msg::FloatingPointRange()
                                  .set__from_value(0.0)
                                  .set__to_value(1.0)}));
  this->declare_parameter<double>("obstacle_scan_duration", 2.0, rcl_interfaces::msg::ParameterDescriptor()
                                  .set__description("Duration to scan each obstacle in seconds."));

  this->declare_parameter<double>("min_obstacle_dimension", 0.3, rcl_interfaces::msg::ParameterDescriptor()
                                  .set__description("Minimum dimension (X, Y, or Z) of the AABB for a valid obstacle (m)."));
  this->declare_parameter<double>("max_obstacle_dimension", 0.7, rcl_interfaces::msg::ParameterDescriptor()
                                  .set__description("Maximum dimension (X, Y, or Z) of the AABB for a valid obstacle (m)."));
//障碍物的扫描角度
  this->declare_parameter<double>("pitch_scan_angle",0.78,rcl_interfaces::msg::ParameterDescriptor()
                                  .set__description("Pitch scan angle (radians)"));
  this->declare_parameter<double>("yaw_scan_angle",0.78,rcl_interfaces::msg::ParameterDescriptor()
                                  .set__description("Yaw scan angle (radians)"));

  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("gimbal_frame", gimbal_frame_);
  this->get_parameter("scan_speed", scan_speed_);
  this->get_parameter("pitch_scan_speed", pitch_scan_speed_);
  this->get_parameter("min_obstacle_size", min_obstacle_size_);
  std::string terrainmap_topic;
  this->get_parameter("terrainmap_topic", terrainmap_topic);
  std::string gimbal_cmd_topic;
  this->get_parameter("gimbal_cmd_topic", gimbal_cmd_topic);
  double voxel_leaf_size;
  this->get_parameter("voxel_leaf_size", voxel_leaf_size);
  this->get_parameter("smoothing_factor", smoothing_factor_);
  this->get_parameter("obstacle_scan_duration", obstacle_scan_duration_);
  this->get_parameter("min_obstacle_dimension", min_obstacle_dimension_);
  this->get_parameter("max_obstacle_dimension", max_obstacle_dimension_);
  this->get_parameter("pitch_scan_angle", pitch_scan_angle_);
  this->get_parameter("yaw_scan_angle", yaw_scan_angle_);


  current_obstacles_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  previous_obstacles_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  voxel_filter_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 初始化扫描相关变量
  target_yaw_ = 0.0;
  target_pitch_ = 0.0;
  scan_offset_yaw_ = 0.0;
  scan_offset_pitch_ = 0.0;
  current_sent_yaw_ = 0.0;
  current_sent_pitch_ = 0.0;
  current_scan_yaw_speed_ = std::abs(scan_speed_); // 初始速度为正
  current_scan_pitch_speed_ = std::abs(pitch_scan_speed_); // 初始速度为正


  terrainmap_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    terrainmap_topic, 10,
    std::bind(&LidarscanNode::terrainmapCallback, this, std::placeholders::_1));

  gimbal_cmd_pub_ = this->create_publisher<auto_aim_interfaces::msg::Send>(
      gimbal_cmd_topic, 10);

  scan_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&LidarscanNode::scanTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Lidarscan node initialized");
}


LidarscanNode::~LidarscanNode()
{
  controlGimbal(0.0, 0.0);
}

void LidarscanNode::terrainmapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(this->get_logger(), "Received null message on terrainmap topic");
    return;
  }
  try {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
      return;
    }
    RCLCPP_DEBUG(this->get_logger(), "Received point cloud with %zu points", cloud->size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_filter_.setInputCloud(cloud);
    voxel_filter_.filter(*filtered_cloud);
    detectObstacles(filtered_cloud);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in terrainmap callback: %s", e.what());
  }
}

void LidarscanNode::detectObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.2);  // m
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(10000);
  ec.setInputCloud(cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);
  bool found_new_valid_obstacle = false;
  pcl::PointCloud<pcl::PointXYZ>::Ptr new_obstacle_list(new pcl::PointCloud<pcl::PointXYZ>());

  for (const auto& cluster : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*cloud, cluster.indices, *cluster_cloud);
      if (cluster_cloud->empty()) continue;
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cluster_cloud, centroid);
      bool height_ok = (centroid[2] > 0.1 && centroid[2] < 2.0); // Check height relative to base_link/odom
      pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
      feature_extractor.setInputCloud(cluster_cloud);
      feature_extractor.compute();
      pcl::PointXYZ min_point_AABB, max_point_AABB;
      feature_extractor.getAABB(min_point_AABB, max_point_AABB);
      float dim_x = max_point_AABB.x - min_point_AABB.x;
      float dim_y = max_point_AABB.y - min_point_AABB.y;
      float dim_z = max_point_AABB.z - min_point_AABB.z;
      float min_dim = std::min({dim_x, dim_y, dim_z});
      float max_dim = std::max({dim_x, dim_y, dim_z});
      bool size_ok = (min_dim >= static_cast<float>(min_obstacle_dimension_) &&
                      max_dim <= static_cast<float>(max_obstacle_dimension_));

      if (height_ok && size_ok) {
        pcl::PointXYZ obstacle_point;
        obstacle_point.x = centroid[0];
        obstacle_point.y = centroid[1];
        obstacle_point.z = centroid[2]; // Store centroid Z for potential future use, though targeting uses Y/X
        if (isNewObstacle(obstacle_point)) {
          new_obstacle_list->push_back(obstacle_point); // Collect all new valid obstacles
          found_new_valid_obstacle = true;
          RCLCPP_INFO(
            this->get_logger(), "New valid obstacle candidate detected at (%.2f, %.2f, %.2f)",
            obstacle_point.x, obstacle_point.y, obstacle_point.z);
        }
      } else {
         RCLCPP_DEBUG(
            this->get_logger(), "Cluster rejected (height_ok: %d, size_ok: %d, dims: %.2fx%.2fx%.2f, min:%.2f, max:%.2f, centroid_z: %.2f)",
            height_ok, size_ok, dim_x, dim_y, dim_z, min_dim, max_dim, centroid[2]);
      }
  } // End for loop over clusters

  { // Lock scope for obstacle list update
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    if (found_new_valid_obstacle) {
        pcl::copyPointCloud(*new_obstacle_list, *current_obstacles_);
        if (current_state_ == ScanState::IDLE) {
            RCLCPP_INFO(this->get_logger(), "New obstacles detected, starting scan sequence.");
            current_state_ = ScanState::TARGETING;
            obstacle_scan_index_ = 0;
        }
    } else if (current_obstacles_->empty()) {
    }
    pcl::copyPointCloud(*current_obstacles_, *previous_obstacles_);
  }
}
void LidarscanNode::scanTimerCallback()
{
  double dt = 0.05; // Approximate time step

  switch (current_state_) {
    case ScanState::IDLE:
      {
        std::lock_guard<std::mutex> lock(obstacles_mutex_);
        if (!current_obstacles_->empty()) {
          RCLCPP_INFO(this->get_logger(), "IDLE: Obstacles available, transitioning to TARGETING.");
          current_state_ = ScanState::TARGETING;
          obstacle_scan_index_ = 0;
        }
      }
      break;
    case ScanState::TARGETING:
      {
        std::lock_guard<std::mutex> lock(obstacles_mutex_);
        if (obstacle_scan_index_ < current_obstacles_->size()) {
          const auto& target_point = current_obstacles_->points[obstacle_scan_index_];
          target_yaw_ = std::atan2(target_point.y, target_point.x);
          target_pitch_ = 0.0;
          RCLCPP_INFO(
            this->get_logger(), "TARGETING: Obstacle %zu at XYZ(%.2f, %.2f, %.2f). Target angles (yaw:%.2f, pitch:%.2f). Moving.",
            obstacle_scan_index_, target_point.x, target_point.y, target_point.z, target_yaw_, target_pitch_);
          current_state_ = ScanState::MOVING_TO_OBSTACLE;
        } else {
          RCLCPP_INFO(this->get_logger(), "TARGETING: No more obstacles or index out of bounds. Restarting obstacle list.");
          obstacle_scan_index_ = 0; // 循环检查障碍物
        }
      }
      break;
    case ScanState::MOVING_TO_OBSTACLE:
      {
        controlGimbal(target_yaw_, target_pitch_);
        double yaw_error = std::abs(target_yaw_ - current_sent_yaw_);
        double pitch_error = std::abs(target_pitch_ - current_sent_pitch_);
        if (yaw_error < 0.05 && pitch_error < 0.05) { // Threshold in radians
          RCLCPP_INFO(this->get_logger(), "MOVING_TO_OBSTACLE: Reached target. Starting scan phase.");
          scan_phase_start_time_ = this->now();
          scan_offset_yaw_ = 0.0; // Reset scan offsets
          scan_offset_pitch_ = 0.0;
          current_scan_yaw_speed_ = std::abs(scan_speed_); // Reset scan speeds/direction
          current_scan_pitch_speed_ = std::abs(pitch_scan_speed_);
          current_state_ = ScanState::SCANNING_OBSTACLE;
        } else {
           RCLCPP_DEBUG(this->get_logger(), "MOVING_TO_OBSTACLE: Moving... Current(%.2f, %.2f), Target(%.2f, %.2f)",
                       current_sent_yaw_, current_sent_pitch_, target_yaw_, target_pitch_);
        }
      }
      break;
    case ScanState::SCANNING_OBSTACLE:
      {
        rclcpp::Duration elapsed_time = this->now() - scan_phase_start_time_;
        if (elapsed_time.seconds() >= 3) { // 持续扫描 3 秒
          RCLCPP_INFO(this->get_logger(), "SCANNING_OBSTACLE: Scan duration %.1fs completed for obstacle %zu.",
                      elapsed_time.seconds(), obstacle_scan_index_);
          obstacle_scan_index_ = (obstacle_scan_index_ + 1) % current_obstacles_->size(); // 循环到下一个障碍物
          current_state_ = ScanState::TARGETING; // 返回到目标状态
        } else {
          scan_offset_yaw_ += current_scan_yaw_speed_ * dt;
          scan_offset_pitch_ += current_scan_pitch_speed_ * dt;

          if (std::abs(scan_offset_yaw_) > yaw_scan_angle_ / 2.0) {
            current_scan_yaw_speed_ = -current_scan_yaw_speed_;
            scan_offset_yaw_ = std::clamp(scan_offset_yaw_, -yaw_scan_angle_ / 2.0, yaw_scan_angle_ / 2.0);
          }
          if (std::abs(scan_offset_pitch_) > pitch_scan_angle_ / 2.0) {
            current_scan_pitch_speed_ = -current_scan_pitch_speed_;
            scan_offset_pitch_ = std::clamp(scan_offset_pitch_, -pitch_scan_angle_ / 2.0, pitch_scan_angle_ / 2.0);
          }

          controlGimbal(target_yaw_ + scan_offset_yaw_, target_pitch_ + scan_offset_pitch_);
          RCLCPP_DEBUG(this->get_logger(), "SCANNING_OBSTACLE: Time %.2f/%.1f. Offset(%.2f, %.2f). Sending(%.2f, %.2f)",
                      elapsed_time.seconds(), 1.5, scan_offset_yaw_, scan_offset_pitch_,
                      target_yaw_ + scan_offset_yaw_, target_pitch_ + scan_offset_pitch_);
        }
      }
      break;
  }
}

bool LidarscanNode::isNewObstacle(const pcl::PointXYZ& point)
{
  if (previous_obstacles_->empty()) {
    return true;
  }
  for (const auto& prev_point : previous_obstacles_->points) {
    double dist = std::sqrt(
      std::pow(point.x - prev_point.x, 2) +
      std::pow(point.y - prev_point.y, 2) +
      std::pow(point.z - prev_point.z, 2));
    if (dist < min_obstacle_size_) {
      return false;
    }
  }
  return true;
}

void LidarscanNode::controlGimbal(double target_yaw, double target_pitch)
{
  rclcpp::Time current_time = this->now();
  double smoothed_yaw = smoothing_factor_ * target_yaw + (1.0 - smoothing_factor_) * current_sent_yaw_;
  double smoothed_pitch = smoothing_factor_ * target_pitch + (1.0 - smoothing_factor_) * current_sent_pitch_;
  current_sent_yaw_ = smoothed_yaw;
  current_sent_pitch_ = smoothed_pitch;

  auto_aim_interfaces::msg::Send cmd;
  cmd.header.stamp = current_time;
  cmd.header.frame_id = gimbal_frame_;
  cmd.yaw = smoothed_yaw;
  cmd.pitch = smoothed_pitch;
  cmd.tracking = false;
  gimbal_cmd_pub_->publish(cmd);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Gimbal Smoothed Control: target(%.3f, %.3f), smoothed(%.3f, %.3f)",
    target_yaw, target_pitch, smoothed_yaw, smoothed_pitch);
}

} // namespace lidarscan

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lidarscan::LidarscanNode)