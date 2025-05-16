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
: Node("lidarscan_node", options)
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
  this->declare_parameter<double>("smoothing_factor", 0.15);
  this->declare_parameter<double>("obstacle_scan_duration", 2.0);
  this->declare_parameter<double>("min_obstacle_dimension", 0.3);
  this->declare_parameter<double>("max_obstacle_dimension", 0.7);
  this->declare_parameter<double>("pitch_scan_angle", 0.78);
  this->declare_parameter<double>("yaw_scan_angle", 0.78);
  this->declare_parameter<double>("idle_scan_speed", 0.7);
  this->declare_parameter<bool>("clockwise_direction", true);
  this->declare_parameter<double>("spin_yaw_period", 5.0);
  this->declare_parameter<double>("spin_pitch_period", 3.0);

  // 获取参数
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("gimbal_frame", gimbal_frame_);
  this->get_parameter("scan_speed", scan_speed_);
  this->get_parameter("pitch_scan_speed", pitch_scan_speed_);
  this->get_parameter("min_obstacle_size", min_obstacle_size_);
  this->get_parameter("smoothing_factor", smoothing_factor_);
  this->get_parameter("obstacle_scan_duration", obstacle_scan_duration_);
  this->get_parameter("min_obstacle_dimension", min_obstacle_dimension_);
  this->get_parameter("max_obstacle_dimension", max_obstacle_dimension_);
  this->get_parameter("pitch_scan_angle", pitch_scan_angle_);
  this->get_parameter("yaw_scan_angle", yaw_scan_angle_);
  this->get_parameter("idle_scan_speed", idle_scan_speed_);
  this->get_parameter("clockwise_direction", clockwise_direction_);
  this->get_parameter("spin_yaw_period", spin_yaw_period_);
  this->get_parameter("spin_pitch_period", spin_pitch_period_);
  
  std::string terrainmap_topic;
  this->get_parameter("terrainmap_topic", terrainmap_topic);
  std::string gimbal_cmd_topic;
  this->get_parameter("gimbal_cmd_topic", gimbal_cmd_topic);
  double voxel_leaf_size;
  this->get_parameter("voxel_leaf_size", voxel_leaf_size);

  // 初始化点云处理
  previous_point_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  voxel_filter_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

  // 初始化TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 创建订阅和发布
  terrainmap_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    terrainmap_topic, 10,
    std::bind(&LidarscanNode::terrainmapCallback, this, std::placeholders::_1));

  gimbal_cmd_pub_ = this->create_publisher<auto_aim_interfaces::msg::Send>(
      gimbal_cmd_topic, 10);

  // 创建定时器
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
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_filter_.setInputCloud(cloud);
    voxel_filter_.filter(*filtered_cloud);
    
    detectAndClassifyObstacles(filtered_cloud);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in terrainmap callback: %s", e.what());
  }
}

void LidarscanNode::detectAndClassifyObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // 使用欧几里得聚类提取障碍物
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.2);  // m
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(10000);
  ec.setInputCloud(cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);
  
  std::vector<Obstacle> new_obstacles;
  
  for (const auto& cluster : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, cluster.indices, *cluster_cloud);
    if (cluster_cloud->empty()) continue;
    
    // 计算质心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster_cloud, centroid);
    
    // 检查高度
    bool height_ok = (centroid[2] > 0.1 && centroid[2] < 2.0);
    
    // 提取AABB特征
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cluster_cloud);
    feature_extractor.compute();
    pcl::PointXYZ min_point_AABB, max_point_AABB;
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    
    // 计算尺寸
    float dim_x = max_point_AABB.x - min_point_AABB.x;
    float dim_y = max_point_AABB.y - min_point_AABB.y;
    float dim_z = max_point_AABB.z - min_point_AABB.z;
    float min_dim = std::min({dim_x, dim_y, dim_z});
    float max_dim = std::max({dim_x, dim_y, dim_z});
    
    // 检查尺寸
    bool size_ok = (min_dim >= static_cast<float>(min_obstacle_dimension_) &&
                   max_dim <= static_cast<float>(max_obstacle_dimension_));
    
    if (height_ok && size_ok) {
      pcl::PointXYZ obstacle_point;
      obstacle_point.x = centroid[0];
      obstacle_point.y = centroid[1];
      obstacle_point.z = centroid[2];
      
      if (isNewObstacle(obstacle_point)) {
        // 计算障碍物的yaw和pitch角度
        double yaw = std::atan2(obstacle_point.y, obstacle_point.x);
        double pitch = 0.0; // 通常pitch设为0，云台会在scan时进行pitch扫描
        
        // 创建障碍物对象并添加到列表
        new_obstacles.emplace_back(obstacle_point, yaw, pitch);
      }
    }
  }
  
  // 更新障碍物列表
  if (!new_obstacles.empty()) {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    // 将新检测到的障碍物添加到列表
    obstacles_.insert(obstacles_.end(), new_obstacles.begin(), new_obstacles.end());
    
    // 更新障碍物周期和ID
    updateObstaclesCycle(current_spin_yaw_);
    assignObstacleIds();
    
    if (current_state_ == ScanState::SPIN && !obstacles_.empty()) {
      // 如果有未扫描的障碍物，查找下一个要扫描的障碍物
      int next_id = findNextUnscannedObstacle();
      if (next_id >= 0) {
        current_obstacle_id_ = next_id;
        RCLCPP_INFO(this->get_logger(), "Next obstacle to scan: ID %d", current_obstacle_id_);
      }
    }
  }
  
  // 更新前一帧点云，用于isNewObstacle比较
  pcl::copyPointCloud(*cloud, *previous_point_cloud_);
}

void LidarscanNode::updateObstaclesCycle(double current_yaw)
{
  // 根据当前yaw值更新障碍物的周期
  for (auto& obstacle : obstacles_) {
    // 判断障碍物是在当前周期还是下一周期
    // 根据旋转方向和当前yaw角度判断
    if (clockwise_direction_) {
      // 顺时针旋转：如果障碍物yaw小于当前yaw（已经扫过），则放入下一周期
      obstacle.cycle = (obstacle.yaw > current_yaw);
    } else {
      // 逆时针旋转：如果障碍物yaw大于当前yaw（已经扫过），则放入下一周期
      obstacle.cycle = (obstacle.yaw < current_yaw);
    }
  }
}

void LidarscanNode::assignObstacleIds()
{
  // 给未扫描的障碍物分配ID，从0开始
  int next_id = 0;
  
  // 先处理当前周期的障碍物
  for (auto& obstacle : obstacles_) {
    if (obstacle.cycle && !obstacle.scanned) {
      obstacle.id = next_id++;
    }
  }
  
  // 再处理下一周期的障碍物
  for (auto& obstacle : obstacles_) {
    if (!obstacle.cycle && !obstacle.scanned) {
      obstacle.id = next_id++;
    }
  }
}

bool LidarscanNode::isNewObstacle(const pcl::PointXYZ& point)
{
  if (previous_point_cloud_->empty()) {
    return true;
  }
  
  for (const auto& prev_point : previous_point_cloud_->points) {
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

int LidarscanNode::findNextUnscannedObstacle()
{
  // 寻找下一个未扫描的障碍物ID
  // 优先扫描当前周期内的障碍物
  
  // 先查找当前周期的障碍物
  for (const auto& obstacle : obstacles_) {
    if (obstacle.cycle && !obstacle.scanned) {
      return obstacle.id;
    }
  }
  
  // 如果当前周期没有未扫描的障碍物，检查下一周期
  for (const auto& obstacle : obstacles_) {
    if (!obstacle.cycle && !obstacle.scanned) {
      return obstacle.id;
    }
  }
  
  return -1; // 没有找到未扫描的障碍物
}

void LidarscanNode::scanTimerCallback()
{
  rclcpp::Time current_time = this->now();
  
  switch (current_state_) {
    case ScanState::SPIN:
    {
      // 在SPIN状态下，云台做连续的旋转运动
      double yaw_direction = clockwise_direction_ ? -1.0 : 1.0;
      double time_sec = current_time.seconds();
      
      // 计算当前的云台角度
      current_spin_yaw_ = yaw_direction * (M_PI * 0.8) * std::sin(2.0 * M_PI * time_sec / spin_yaw_period_);
      current_spin_pitch_ = (pitch_scan_angle_ / 2.0) * std::sin(2.0 * M_PI * time_sec / spin_pitch_period_);
      
      // 检查是否有未扫描的障碍物
      bool has_unscanned_obstacle = false;
      {
        std::lock_guard<std::mutex> lock(obstacles_mutex_);
        
        // 更新障碍物周期
        updateObstaclesCycle(current_spin_yaw_);
        assignObstacleIds();
        
        // 检查是否有障碍物需要扫描
        int next_id = findNextUnscannedObstacle();
        if (next_id >= 0) {
          has_unscanned_obstacle = true;
          current_obstacle_id_ = next_id;
          
          // 找到对应的障碍物
          for (auto& obstacle : obstacles_) {
            if (obstacle.id == current_obstacle_id_) {
              // 切换到SCAN状态
              current_state_ = ScanState::SCAN;
              scan_start_time_ = current_time;
              RCLCPP_INFO(this->get_logger(), "SPIN: Transitioning to SCAN for obstacle ID %d at yaw=%.2f",
                          current_obstacle_id_, obstacle.yaw);
              break;
            }
          }
        }
      }
      
      if (!has_unscanned_obstacle) {
        // 如果没有障碍物需要扫描，继续执行全局旋转
        controlGimbal(current_spin_yaw_, current_spin_pitch_);
        RCLCPP_DEBUG(this->get_logger(), "SPIN: Global scan at yaw=%.2f, pitch=%.2f", 
                     current_spin_yaw_, current_spin_pitch_);
      }
      break;
    }
    
    case ScanState::SCAN:
    {
      rclcpp::Duration elapsed_time = current_time - scan_start_time_;
      
      // 检查是否完成扫描
      if (elapsed_time.seconds() >= obstacle_scan_duration_) {
        // 扫描完成，将障碍物标记为已扫描
        {
          std::lock_guard<std::mutex> lock(obstacles_mutex_);
          for (auto& obstacle : obstacles_) {
            if (obstacle.id == current_obstacle_id_) {
              obstacle.scanned = true;
              RCLCPP_INFO(this->get_logger(), "SCAN: Completed scan for obstacle ID %d, marked as scanned", 
                          current_obstacle_id_);
              break;
            }
          }
        }
        
        // 返回SPIN状态
        current_state_ = ScanState::SPIN;
        RCLCPP_INFO(this->get_logger(), "SCAN: Returning to SPIN state");
        
      } else {
        // 继续扫描当前障碍物
        double target_yaw = 0.0;
        double target_pitch = 0.0;
        
        {
          std::lock_guard<std::mutex> lock(obstacles_mutex_);
          for (const auto& obstacle : obstacles_) {
            if (obstacle.id == current_obstacle_id_) {
              target_yaw = obstacle.yaw;
              target_pitch = obstacle.pitch;
              break;
            }
          }
        }
        
        // 计算扫描偏移量
        double yaw_direction = clockwise_direction_ ? -1.0 : 1.0;
        double yaw_offset = yaw_direction * (yaw_scan_angle_ / 2.0) * 
                          std::sin(2.0 * M_PI * elapsed_time.seconds() / 1.5);
        double pitch_offset = (pitch_scan_angle_ / 2.0) * 
                             std::sin(2.0 * M_PI * elapsed_time.seconds() / 1.0);
        
        // 控制云台
        controlGimbal(target_yaw + yaw_offset, target_pitch + pitch_offset);
        
        RCLCPP_DEBUG(this->get_logger(), "SCAN: Obstacle ID %d, Time %.2f/%.1f, Target(%.2f, %.2f), Offset(%.2f, %.2f)",
                    current_obstacle_id_, elapsed_time.seconds(), obstacle_scan_duration_,
                    target_yaw, target_pitch, yaw_offset, pitch_offset);
      }
      break;
    }
  }
}

void LidarscanNode::controlGimbal(double target_yaw, double target_pitch)
{
  // 平滑云台控制
  double smoothed_yaw = smoothing_factor_ * target_yaw + (1.0 - smoothing_factor_) * current_sent_yaw_;
  double smoothed_pitch = smoothing_factor_ * target_pitch + (1.0 - smoothing_factor_) * current_sent_pitch_;
  
  current_sent_yaw_ = smoothed_yaw;
  current_sent_pitch_ = smoothed_pitch;
  
  // 发布云台控制命令
  auto_aim_interfaces::msg::Send cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = gimbal_frame_;
  cmd.yaw = smoothed_yaw;
  cmd.pitch = smoothed_pitch;
  cmd.tracking = false;
  gimbal_cmd_pub_->publish(cmd);
  
  RCLCPP_DEBUG(this->get_logger(), "Gimbal Command: yaw=%.3f, pitch=%.3f", smoothed_yaw, smoothed_pitch);
}

}  // namespace lidarscan

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lidarscan::LidarscanNode)