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
#include <chrono>

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
#include "lidarscan/pc_process.hpp"  // 新增头文件引用

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

    rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr gimbal_cmd_pub_;

    rclcpp::TimerBase::SharedPtr scan_timer_;
    void scanTimerCallback();
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 点云处理器
    std::unique_ptr<PointCloudProcessor> pc_processor_;

    // 障碍物类定义
    class Obstacle {
    public:
        int id;              // 障碍物ID
        double yaw;          // 障碍物偏航角
        double pitch;        // 障碍物俯仰角
        bool cycle;          // true为当前周期，false为下一周期
        bool scanned;        // 标记是否已扫描过
        pcl::PointXYZ point; // 障碍物坐标
        int cycle_count;     // 经历的周期计数，超过限制后删除

        Obstacle(const pcl::PointXYZ& p, double y, double p_angle) 
            : id(-1), yaw(y), pitch(p_angle), cycle(true), scanned(false), 
              point(p), cycle_count(0) {}
    };

    // 障碍物相关
    std::vector<Obstacle> obstacles_;
    std::mutex obstacles_mutex_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
    int current_obstacle_id_{0};   
    int max_cycle_count_{3};       // 障碍物最大周期计数
    
    // 云台状态相关
    ScanState current_state_{ScanState::SPIN};
    double current_spin_yaw_{0.0};  
    double current_spin_pitch_{0.0};
    double current_sent_yaw_{0.0};  
    double current_sent_pitch_{0.0};
    
    // 周期管理
    bool cycle_completed_{false};  
    double cycle_start_yaw_{0.0};  
    bool is_first_cycle_{true};    
    
    // 扫描相关参数
    rclcpp::Time scan_start_time_; 
    double scan_speed_{0.3};       
    double pitch_scan_speed_{0.25};
    double pitch_scan_angle_{0.78};
    double yaw_scan_angle_{0.78};  
    double spin_yaw_period_{5.0};  
    double spin_pitch_period_{3.0};
    double obstacle_scan_duration_{2.0};
    double smoothing_factor_{0.15};
    bool clockwise_direction_{true};
    
    // 点云处理相关
    double min_obstacle_size_{0.1};
    double min_obstacle_dimension_{0.3};
    double max_obstacle_dimension_{0.7};
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;

    // 坐标系
    std::string map_frame_;
    std::string base_frame_;
    std::string gimbal_frame_;
    
    // 方法
    void processDetectedObstacles(const std::vector<ObstaclePoint>& detected_obstacles);
    void detectAndClassifyObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void updateObstaclesCycle(double current_yaw);
    void assignObstacleIds();
    int findNextUnscannedObstacle();
    void resetObstaclesAfterCycle();
    bool isCompleteCycle(double current_yaw);
    bool isNewObstacle(const pcl::PointXYZ& point);
    void cleanupOldObstacles();
    void controlGimbal(double yaw, double pitch);
    void logObstaclesState(const std::string& prefix);
};

}  // namespace lidarscan

#endif  // LIDARSCAN__LIDARSCAN_HPP_