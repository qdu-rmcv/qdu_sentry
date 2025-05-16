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

namespace lidarscan {

enum scan_state {
    SPIN,
    SCAN,
};

class LidarscanNode : public rclcpp::Node
{
public:
    LidarscanNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~LidarscanNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrainmap_sub_;
    void terrainmapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr gimbal_cmd_pub_;

    rclcpp::TimerBase::SharedPtr scan_timer_;
    void scanTimerCallback();

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    class Obstacle{
    public:
        int id;
        double yaw;
        double pitch;
        int cycle;
    };
    
    std::vector<Obstacle> obstacles_;

    double spin_yaw_speed_;         //云台水平速度
    double spin_pitch_speed_;       //云台点头速度

    int current_period_;            //当前周期
    int max_obstacle_period_;       //障碍物的存在周期
    
};
}
#endif  // LIDARSCAN__LIDARSCAN_HPP_