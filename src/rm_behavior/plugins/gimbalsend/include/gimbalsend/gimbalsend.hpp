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

#ifndef TRACKERSEND__TRACKERSEND_HPP_
#define TRACKERSEND__TRACKERSEND_HPP_

#include <optional>
#include <mutex>
#include <vector>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "auto_aim_interfaces/msg/send.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"

namespace gimbalsend
{

class GimbalsendNode : public rclcpp::Node
{
public:
  explicit GimbalsendNode(const rclcpp::NodeOptions & options);

private:
  // 回调函数
  void trackerCallback(const auto_aim_interfaces::msg::Send::SharedPtr msg);
  void lidarCallback(const auto_aim_interfaces::msg::Send::SharedPtr msg);
  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg);
  void publishTimerCallback();

  // 订阅者
  rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr tracker_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr lidar_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;

  // 发布者
  rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr gimbal_pub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // 数据保护
  std::mutex data_mutex_;

  // 状态变量
  bool enemy_detected_;
  bool last_enemy_detected_;
  std::optional<auto_aim_interfaces::msg::Send> last_tracker_msg_;
  std::optional<auto_aim_interfaces::msg::Send> last_lidar_msg_;
  rclcpp::Time last_armor_time_;
  rclcpp::Time mode_switch_time_;

  // 配置参数
  std::vector<int64_t> expected_armor_ids_;
  double max_distance_;
  double detection_timeout_;
  double scan_transition_delay_;

  // 日志
  rclcpp::Logger logger_;
};

}  // namespace gimbalsend

#endif  // TRACKERSEND__TRACKERSEND_HPP_
