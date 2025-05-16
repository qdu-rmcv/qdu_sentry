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
#include "lidarscan/pc_process.hpp"  // 添加头文件引用
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
  this->declare_parameter<bool>("clockwise_direction", true);
  this->declare_parameter<double>("spin_yaw_period", 5.0);
  this->declare_parameter<double>("spin_pitch_period", 3.0);
  this->declare_parameter<int>("max_cycle_count", 3);

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
  this->get_parameter("clockwise_direction", clockwise_direction_);
  this->get_parameter("spin_yaw_period", spin_yaw_period_);
  this->get_parameter("spin_pitch_period", spin_pitch_period_);
  this->get_parameter("max_cycle_count", max_cycle_count_);
  
  std::string terrainmap_topic;
  this->get_parameter("terrainmap_topic", terrainmap_topic);
  std::string gimbal_cmd_topic;
  this->get_parameter("gimbal_cmd_topic", gimbal_cmd_topic);
  double voxel_leaf_size;
  this->get_parameter("voxel_leaf_size", voxel_leaf_size);

  // 初始化点云处理器
  pc_processor_ = std::make_unique<PointCloudProcessor>(this->get_logger());
  pc_processor_->configure(voxel_leaf_size, 
                         min_obstacle_dimension_, 
                         max_obstacle_dimension_,
                         min_obstacle_size_);

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

  // 初始化周期相关变量
  cycle_completed_ = false;
  cycle_start_yaw_ = 0.0;
  is_first_cycle_ = true;

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
    
    // 使用点云处理器过滤点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = pc_processor_->filterCloud(cloud);
    
    // 使用点云处理器检测障碍物
    std::vector<ObstaclePoint> detected_obstacles = pc_processor_->detectObstacles(filtered_cloud);
    
    // 处理检测到的障碍物
    processDetectedObstacles(detected_obstacles);
    
    // 更新前一帧点云
    pc_processor_->setPreviousCloud(filtered_cloud);
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in terrainmap callback: %s", e.what());
  }
}

void LidarscanNode::processDetectedObstacles(const std::vector<ObstaclePoint>& detected_obstacles)
{
  if (detected_obstacles.empty()) {
    return;
  }
  
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  
  // 为每个检测到的障碍物创建Obstacle对象
  for (const auto& obs_point : detected_obstacles) {
    obstacles_.emplace_back(obs_point.point, obs_point.yaw, obs_point.pitch);
  }
  
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

void LidarscanNode::updateObstaclesCycle(double current_yaw)
{
  // 检查是否完成了一个完整的周期
  if (isCompleteCycle(current_yaw)) {
    RCLCPP_INFO(this->get_logger(), "Completed full rotation cycle");
    
    // 更新所有障碍物的周期计数
    for (auto& obstacle : obstacles_) {
      if (!obstacle.scanned) {
        obstacle.cycle_count++;
      }
    }
    
    // 清理旧的障碍物
    cleanupOldObstacles();
    
    // 更新周期起始点
    cycle_start_yaw_ = current_yaw;
    cycle_completed_ = true;
    is_first_cycle_ = false;
    
    // 重置障碍物状态
    resetObstaclesAfterCycle();
    
    RCLCPP_INFO(this->get_logger(), "After cycle completion: %zu obstacles remain", obstacles_.size());
  }

  // 根据当前云台yaw值和障碍物yaw值确定周期属性
  for (auto& obstacle : obstacles_) {
    // 已经扫描过的障碍物设为下一个周期
    if (obstacle.scanned) {
      obstacle.cycle = false;
      continue;
    }
    
    // 根据旋转方向和云台当前位置决定障碍物所属周期
    if (clockwise_direction_) {
      // 顺时针旋转：如果障碍物yaw小于当前yaw(已扫过)，放入下一周期
      obstacle.cycle = obstacle.yaw > current_yaw;
    } else {
      // 逆时针旋转：如果障碍物yaw大于当前yaw(已扫过)，放入下一周期
      obstacle.cycle = obstacle.yaw < current_yaw;
    }
  }
}

void LidarscanNode::cleanupOldObstacles()
{
  // 移除超过周期限制的障碍物
  size_t original_size = obstacles_.size();
  
  obstacles_.erase(
    std::remove_if(obstacles_.begin(), obstacles_.end(),
                  [this](const Obstacle& o) { return o.cycle_count >= max_cycle_count_; }),
    obstacles_.end());
  
  size_t removed_count = original_size - obstacles_.size();
  if (removed_count > 0) {
    RCLCPP_INFO(this->get_logger(), "Removed %zu obstacles exceeding cycle limit", removed_count);
  }
}

bool LidarscanNode::isCompleteCycle(double current_yaw)
{
  static bool cycle_started = false;
  static bool halfway_point = false;
  static double last_yaw = current_yaw;
  
  // 计算yaw变化
  double yaw_change = current_yaw - last_yaw;
  if (yaw_change > M_PI) yaw_change -= 2 * M_PI;
  if (yaw_change < -M_PI) yaw_change += 2 * M_PI;
  
  // 检测yaw方向变化，用于识别正弦波的极值点
  bool is_yaw_direction_change = false;
  if (clockwise_direction_) {
    is_yaw_direction_change = (yaw_change > 0.1 && last_yaw < -0.8 * M_PI);
  } else {
    is_yaw_direction_change = (yaw_change < -0.1 && last_yaw > 0.8 * M_PI);
  }
  
  last_yaw = current_yaw;
  
  // 使用正弦波的特性来检测周期完成
  if (!cycle_started) {
    // 周期开始
    if (std::abs(current_yaw) > 0.5 * M_PI) {
      cycle_started = true;
      RCLCPP_DEBUG(this->get_logger(), "Cycle detection started at yaw=%.2f", current_yaw);
    }
    return false;
  } else if (!halfway_point) {
    // 检测是否达到半周期
    if (std::abs(current_yaw) < 0.3) {
      halfway_point = true;
      RCLCPP_DEBUG(this->get_logger(), "Halfway point detected at yaw=%.2f", current_yaw);
    }
    return false;
  } else {
    // 检测是否完成一个完整周期
    if (is_yaw_direction_change) {
      // 重置状态变量
      cycle_started = false;
      halfway_point = false;
      RCLCPP_DEBUG(this->get_logger(), "Full cycle completed, returning to start position");
      return true;
    }
    return false;
  }
}

void LidarscanNode::resetObstaclesAfterCycle()
{
  // 重置所有障碍物的扫描状态
  for (auto& obstacle : obstacles_) {
    obstacle.scanned = false;
  }
  
  // 重新分配ID
  assignObstacleIds();
}

void LidarscanNode::assignObstacleIds()
{
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

int LidarscanNode::findNextUnscannedObstacle()
{
  // 日志记录障碍物状态
  logObstaclesState("Finding next obstacle");
  
  // 优先扫描当前周期内的障碍物
  for (const auto& obstacle : obstacles_) {
    if (obstacle.cycle && !obstacle.scanned) {
      RCLCPP_INFO(this->get_logger(), "Found unscanned obstacle in current cycle: ID %d", obstacle.id);
      return obstacle.id;
    }
  }
  
  // 如果当前周期没有未扫描的障碍物，检查下一周期
  for (const auto& obstacle : obstacles_) {
    if (!obstacle.cycle && !obstacle.scanned) {
      RCLCPP_INFO(this->get_logger(), "Found unscanned obstacle in next cycle: ID %d", obstacle.id);
      return obstacle.id;
    }
  }
  
  RCLCPP_DEBUG(this->get_logger(), "No unscanned obstacles found");
  return -1;
}

void LidarscanNode::logObstaclesState(const std::string& prefix)
{
  std::stringstream ss;
  ss << prefix << " - Obstacles state: ";
  for (const auto& o : obstacles_) {
    ss << "[id=" << o.id << ", yaw=" << o.yaw << ", cycle=" << (o.cycle ? "true" : "false") 
       << ", scanned=" << (o.scanned ? "true" : "false") << ", cycle_count=" << o.cycle_count << "] ";
  }
  RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
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
      
      // 计算当前的云台角度，用于平滑的周期性扫描
      double new_spin_yaw = yaw_direction * (M_PI * 0.8) * std::sin(2.0 * M_PI * time_sec / spin_yaw_period_);
      double new_spin_pitch = (pitch_scan_angle_ / 2.0) * std::sin(2.0 * M_PI * time_sec / spin_pitch_period_);
      
      // 记录前一帧的yaw值，用于检测周期完成
      current_spin_yaw_ = new_spin_yaw;
      current_spin_pitch_ = new_spin_pitch;
      
      // 检查是否有未扫描的障碍物
      bool has_unscanned_obstacle = false;
      {
        std::lock_guard<std::mutex> lock(obstacles_mutex_);
        
        // 更新障碍物周期
        updateObstaclesCycle(current_spin_yaw_);
        
        // 处理周期完成事件
        if (cycle_completed_) {
          RCLCPP_INFO(this->get_logger(), "Processing cycle completion, %zu obstacles in list", obstacles_.size());
          cycle_completed_ = false;
        }
        
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
          bool found = false;
          for (auto& obstacle : obstacles_) {
            if (obstacle.id == current_obstacle_id_) {
              obstacle.scanned = true;
              obstacle.cycle = false;
              found = true;
              RCLCPP_INFO(this->get_logger(), "SCAN: Completed scan for obstacle ID %d, marked as scanned", 
                          current_obstacle_id_);
              break;
            }
          }
          
          if (!found) {
            RCLCPP_WARN(this->get_logger(), "Could not find obstacle ID %d to mark as scanned", current_obstacle_id_);
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
        
        // 计算扫描偏移量，以障碍物为中心进行扫描
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