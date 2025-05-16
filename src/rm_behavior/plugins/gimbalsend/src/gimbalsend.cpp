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

#include "gimbalsend/gimbalsend.hpp"

namespace gimbalsend
{

GimbalsendNode::GimbalsendNode(const rclcpp::NodeOptions & options)
: Node("gimbalsend_node", options),
  enemy_detected_(false),
  last_enemy_detected_(false),
  last_tracker_msg_(std::nullopt),
  last_lidar_msg_(std::nullopt),
  mode_switch_time_(this->now()),
  logger_(get_logger())
{
  this->declare_parameter<std::string>("lidarscan_topic", "/lidarscan");
  this->declare_parameter<std::string>("tracker_topic", "/tracker/send");
  this->declare_parameter<std::string>("gimbal_pub_topic", "/gimbalcontrol");
  this->declare_parameter<std::string>("armors_topic", "/detector/armors");
  this->declare_parameter<std::vector<int64_t>>("armor_id", std::vector<int64_t>{1, 2, 3, 4, 5, 6, 7, 8, 9});
  this->declare_parameter<double>("max_distance", 8.0);
  this->declare_parameter<double>("transition_time", 1.5);//自瞄丢失时间

  std::string lidarscan_topic = this->get_parameter("lidarscan_topic").as_string();
  std::string tracker_topic = this->get_parameter("tracker_topic").as_string();
  std::string gimbal_pub_topic = this->get_parameter("gimbal_pub_topic").as_string();
  std::string armors_topic = this->get_parameter("armors_topic").as_string();
  
  expected_armor_ids_ = this->get_parameter("armor_id").as_integer_array();
  max_distance_ = this->get_parameter("max_distance").as_double();
  detection_timeout_ = this->get_parameter("transition_time").as_double();
  scan_transition_delay_ = detection_timeout_;  // 使用相同的值

  last_armor_time_ = this->now();

  // 初始化订阅者
  lidar_sub_ = this->create_subscription<auto_aim_interfaces::msg::Send>(
    lidarscan_topic, rclcpp::SensorDataQoS(),
    std::bind(&GimbalsendNode::lidarCallback, this, std::placeholders::_1));
    
  tracker_sub_ = this->create_subscription<auto_aim_interfaces::msg::Send>(
    tracker_topic, rclcpp::SensorDataQoS(),
    std::bind(&GimbalsendNode::trackerCallback, this, std::placeholders::_1));
    
  armors_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
    armors_topic, rclcpp::SensorDataQoS(),
    std::bind(&GimbalsendNode::armorsCallback, this, std::placeholders::_1));

  // 初始化发布者
  gimbal_pub_ = this->create_publisher<auto_aim_interfaces::msg::Send>(
    gimbal_pub_topic, rclcpp::SensorDataQoS());

  // 初始化定时器
  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(30),
    std::bind(&GimbalsendNode::publishTimerCallback, this));

  RCLCPP_INFO(logger_, "GimbalsendNode 初始化完成. 使用话题: lidarscan=%s, tracker=%s, control=%s, gimbalsend=%s",
              lidarscan_topic.c_str(), tracker_topic.c_str(), armors_topic.c_str(), gimbal_pub_topic.c_str());
}

void GimbalsendNode::trackerCallback(const auto_aim_interfaces::msg::Send::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_tracker_msg_ = *msg;
  RCLCPP_INFO(logger_, "自瞄消息");
}

void GimbalsendNode::lidarCallback(const auto_aim_interfaces::msg::Send::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_lidar_msg_ = *msg;
  RCLCPP_INFO(logger_, "扫描消息");
}

void GimbalsendNode::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg)
{
  bool detected = false;
  
  if (!msg->armors.empty()) {
    for (const auto & armor : msg->armors) {
      // 计算敌人距离
      float distance_to_enemy = std::hypot(armor.pose.position.x, armor.pose.position.y);

      if (armor.number.empty()) {
        continue;
      }

      try {
        int armor_id = std::stoi(armor.number);
        // 检查是否为期望的装甲板ID
        const bool is_armor_id_match =
          std::find(expected_armor_ids_.begin(), expected_armor_ids_.end(), armor_id) !=
          expected_armor_ids_.end();

        const bool is_within_distance = (distance_to_enemy <= max_distance_);

        if (is_armor_id_match && is_within_distance) {
          detected = true;
          break;
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(logger_, "装甲板ID解析错误: %s", armor.number.c_str());
      }
    }
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  enemy_detected_ = detected;
  last_armor_time_ = this->now();
  
  RCLCPP_INFO(logger_, "%s", detected ? "检测到敌人" : "未检测到敌人");
}

void GimbalsendNode::publishTimerCallback()
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // 检查超时
  bool is_detection_valid = 
    (this->now() - last_armor_time_).seconds() < detection_timeout_;

  // 保存上一帧状态
  bool previous_detected = enemy_detected_;

  if (!is_detection_valid) {
    enemy_detected_ = false;
  }

  // 检测状态转换：自瞄 → 扫描
  if (previous_detected && !enemy_detected_) {
    mode_switch_time_ = this->now();
    RCLCPP_INFO(logger_, "模式切换: 自瞄 → 扫描 (延迟: %.1f秒)", scan_transition_delay_);
  }

  // 决定发布什么数据
  if (enemy_detected_) {
    // 检测到敌人，使用自瞄数据
    if (last_tracker_msg_.has_value()) {
      RCLCPP_INFO(logger_, "发布 tracker 数据");
      gimbal_pub_->publish(last_tracker_msg_.value());
    } else {
      RCLCPP_WARN(logger_, "检测到敌人，但没有可用的 tracker 消息");
    }
  } else {
    // 没有检测到敌人
    double time_since_switch = (this->now() - mode_switch_time_).seconds();
    
    // 如果刚从自瞄切换到扫描且在延迟时间内，保持使用之前的自瞄数据
    if (previous_detected && time_since_switch < scan_transition_delay_ && last_tracker_msg_.has_value()) {
      RCLCPP_INFO(logger_, "保持自瞄方向: %.1f/%.1f秒", time_since_switch, scan_transition_delay_);
      gimbal_pub_->publish(last_tracker_msg_.value());
    } else {
      // 否则使用扫描数据
      if (last_lidar_msg_.has_value()) {
        RCLCPP_INFO(logger_, "发布 lidar 数据");
        gimbal_pub_->publish(last_lidar_msg_.value());
      }
    }
  }
  
  // 更新状态记录
  last_enemy_detected_ = enemy_detected_;
}

} // namespace gimbalsend

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gimbalsend::GimbalsendNode)

