// Copyright 2025 Jquark
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
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
#include <optional>
#include <algorithm> // 添加算法库，用于std::sort

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "auto_aim_interfaces/msg/send.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/chassis.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "Eigen/Core"

namespace lidarscan
{

    enum scan_state
    {
        SPIN,         // 旋转状态
        SCAN,         // 障碍物扫描状态
        DAMAGED_SCAN, // 装甲板击打反馈
        TRACKING,     // 自瞄追踪状态
    };
    // 状态转换优先级：TRACKING > DAMAGED_SCAN > SCAN > SPIN

    class LidarscanNode : public rclcpp::Node
    {
    public:
        LidarscanNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~LidarscanNode();

    private:
        // 点云处理相关
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrainmap_sub_;
        void terrainmapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        double min_bounding_box;
        double max_bounding_box;
        std::string terrain_map_topic_;

        // 运动控制相关
        rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr gimbal_cmd_pub_;
        std::string gimbal_cmd_topic_;

        rclcpp::TimerBase::SharedPtr scan_timer_;
        void scanTimerCallback();
        rclcpp::Time last_time_;
        double dt_;

        // Chassis相关
        rclcpp::Subscription<auto_aim_interfaces::msg::Chassis>::SharedPtr chassis_sub_;
        void chassisCallback(const auto_aim_interfaces::msg::Chassis::SharedPtr msg);
        double calculateArmorYaw(uint8_t armor_id);
        bool damaged_armor_detected_;
        uint8_t current_damaged_armor_id_;
        double chassis_yaw_offset_;
        std::string chassis_topic_;

        rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr tracker_sub_;
        void trackerCallback(const auto_aim_interfaces::msg::Send::SharedPtr msg);
        std::optional<auto_aim_interfaces::msg::Send> last_tracker_msg_;
        rclcpp::Time last_tracker_time_;
        std::string tracker_topic_;

        rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;
        void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg);
        std::vector<int64_t> expected_armor_ids_;
        double max_distance_;
        bool enemy_detected_;
        rclcpp::Time last_armor_time_;
        std::string armors_topic_;

        std::mutex data_mutex_;

        // 云台控制
        scan_state current_state_;
        void updateGimbalCommand();
        void spinState();
        void scanState();
        void damagedScanState();
        void trackingState();
        bool isYawNearObstacle();
        double closest_obstacle_yaw_;
        double closest_obstacle_distance_; // 记录最近障碍物的角度

        void gimbalsend();
        void checkStateTransition();

        double target_yaw_;
        double target_pitch_;
        double current_yaw_;
        double current_pitch_;
        double pitch_more_speed_;
        double pitch_less_speed_;

        bool tracking_mode_;

        double yaw_smooth_factor_;

        double pitch_smooth_factor_;
        double last_yaw_cmd_;
        double last_pitch_cmd_;

        std::queue<double> yaw_cmd_queue_;
        std::queue<double> pitch_cmd_queue_;
        int smooth_window_size_;
        double updateSmoothedValue(std::queue<double> &cmd_queue, double new_value);

        rclcpp::Time mode_switch_time_;
        double scan_transition_delay_;
        int state_switch_counter_;    
        int max_switch_count_;        
        double hysteresis_threshold_; 

        // 平滑过渡相关变量
        bool in_transition_to_scan_;     
        double transition_progress_;     
        double transition_speed_;        
        rclcpp::Time tracking_exit_time_;
        double tracking_lose_yaw_;

        double scan_phase_;
        int current_period;
        int max_scan_period_;

        std::vector<double> obstacle_yaws_;

        double spin_yaw_speed_;
        double spin_pitch_speed_;
        double min_pitch_;
        double max_pitch_;
        double scan_theta_;

        double state_switch_threshold_;
        int min_scan_points_;

        // 日志
        rclcpp::Logger logger_;

        double time_since_exit;
    };
}
#endif // LIDARSCAN__LIDARSCAN_HPP_