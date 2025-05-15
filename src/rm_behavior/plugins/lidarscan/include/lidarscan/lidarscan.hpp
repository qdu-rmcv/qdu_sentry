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

#ifndef LIDARSCAN__LIDARSCAN_HPP_
#define LIDARSCAN__LIDARSCAN_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <queue>
#include <chrono> // For timing

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "auto_aim_interfaces/msg/send.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/common/centroid.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace lidarscan {

// 定义扫描状态
enum class ScanState {
    IDLE,               // 空闲
    TARGETING,          // 正在选择下一个目标
    MOVING_TO_OBSTACLE, // 正在移动到目标障碍物
    SCANNING_OBSTACLE   // 正在对目标障碍物进行详细扫描
};

class LidarscanNode : public rclcpp::Node {
public:
    LidarscanNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~LidarscanNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrainmap_sub_;
    void terrainmapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;
    void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg);

    rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr
        gimbal_cmd_pub_;

    rclcpp::TimerBase::SharedPtr scan_timer_;
    void scanTimerCallback();

    void detectObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    bool isNewObstacle(const pcl::PointXYZ& point);

    void controlGimbal(double yaw, double pitch);
    bool isEnemyDetected(const auto_aim_interfaces::msg::Armors::SharedPtr msg);

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_obstacles_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_obstacles_;
    std::mutex obstacles_mutex_;

    // 参数
    double scan_speed_;             // 扫描摆动速度 (yaw)
    double pitch_scan_speed_;       // 扫描摆动速度 (pitch)
    double min_obstacle_size_;      // 用于 isNewObstacle
    double max_scan_angle_;         // 旧参数，可能不再直接使用，保留以防万一
    double max_pitch_angle_;        // 旧参数，可能不再直接使用，保留以防万一
    std::vector<int64_t> expected_armor_ids_;
    float max_detect_distance_;
    double smoothing_factor_;       // 平滑因子
    double obstacle_scan_duration_; // 单个障碍物扫描持续时间
    double min_obstacle_dimension_; // AABB 最小尺寸限制
    double max_obstacle_dimension_; // AABB 最大尺寸限制
    double pitch_scan_angle_;       // 扫描摆动角度 (pitch)
    double yaw_scan_angle_;         // 扫描摆动角度 (yaw)

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;

    std::string map_frame_;
    std::string base_frame_;
    std::string gimbal_frame_;
    std::string armors_topic_;

    // 状态变量
    ScanState current_state_;       // 当前状态机状态
    bool enemy_detected_;           // 是否检测到敌人
    size_t obstacle_scan_index_;    // 当前扫描的障碍物索引
    rclcpp::Time scan_phase_start_time_; // 当前扫描阶段开始时间

    // 目标和扫描控制变量
    double target_yaw_;              // 当前目标障碍物的中心 yaw
    double target_pitch_;            // 当前目标障碍物的中心 pitch
    double scan_offset_yaw_;         // 当前扫描摆动的 yaw 偏移量
    double scan_offset_pitch_;       // 当前扫描摆动的 pitch 偏移量
    double current_sent_yaw_;        // 当前已发送的 yaw 值 (用于平滑和状态判断)
    double current_sent_pitch_;      // 当前已发送的 pitch 值 (用于平滑和状态判断)
    double current_scan_yaw_speed_;  // 当前扫描摆动速度 (yaw, 带方向)
    double current_scan_pitch_speed_;// 当前扫描摆动速度 (pitch, 带方向)

};

}  // namespace lidarscan

#endif  // LIDARSCAN__LIDARSCAN_HPP_