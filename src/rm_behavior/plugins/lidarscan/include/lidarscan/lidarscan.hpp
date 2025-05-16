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

// 扫描状态枚举
enum class ScanState {
    SPIN,           // 云台的旋转状态
    SCAN,           // 云台的扫描状态
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

    // 障碍物类定义
    class Obstacle {
    public:
        int id;              // 障碍物ID
        double yaw;          // 障碍物偏航角
        double pitch;        // 障碍物俯仰角
        bool cycle;          // true为当前周期，false为下一周期
        bool scanned;        // 标记是否已扫描过
        pcl::PointXYZ point; // 障碍物坐标

        Obstacle(const pcl::PointXYZ& p, double y, double p_angle) 
            : id(-1), yaw(y), pitch(p_angle), cycle(true), scanned(false), point(p) {}
    };

    // 障碍物相关
    std::vector<Obstacle> obstacles_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_point_cloud_;
    int current_obstacle_id_{0};    // 当前处理的障碍物ID
    
    // 云台状态相关
    ScanState current_state_{ScanState::SPIN};
    double current_spin_yaw_{0.0};  // 当前旋转的yaw值
    double current_spin_pitch_{0.0}; // 当前旋转的pitch值
    double current_sent_yaw_{0.0};   // 当前已发送的实际yaw值
    double current_sent_pitch_{0.0}; // 当前已发送的实际pitch值
    
    // 扫描相关参数
    rclcpp::Time scan_start_time_;  // 扫描开始时间
    double scan_speed_{0.3};        // 扫描速度
    double pitch_scan_speed_{0.25}; // pitch扫描速度
    double pitch_scan_angle_{0.78}; // pitch扫描角度
    double yaw_scan_angle_{0.78};   // yaw扫描角度
    double idle_scan_speed_{0.5};   // 空闲扫描速度
    double spin_yaw_period_{5.0};   // 旋转周期
    double spin_pitch_period_{3.0}; // pitch旋转周期
    double obstacle_scan_duration_{2.0}; // 障碍物扫描持续时间
    double smoothing_factor_{0.15}; // 平滑因子
    bool clockwise_direction_{true}; // 顺时针方向
    
    // 点云处理相关
    double min_obstacle_size_{0.1}; // 障碍物最小尺寸
    double min_obstacle_dimension_{0.3}; // 最小障碍物维度
    double max_obstacle_dimension_{0.7}; // 最大障碍物维度
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;

    // 坐标系
    std::string map_frame_;
    std::string base_frame_;
    std::string gimbal_frame_;
    
    // 方法
    void detectAndClassifyObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void updateObstaclesCycle(double current_yaw);
    void assignObstacleIds();
    int findNextUnscannedObstacle();
};

}  // namespace lidarscan

#endif  // LIDARSCAN__LIDARSCAN_HPP_