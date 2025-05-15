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

#include "referee_interfaces/msg/basic_hp.hpp"
#include "rm_behavior/plugins/condition/home_check.hpp"

namespace rm_behavior {

HomeCheckCondition::HomeCheckCondition(const std::string &name,
                                           const BT::NodeConfig &config)
    : BT::ConditionNode(name, config) {}

BT::NodeStatus HomeCheckCondition::tick() {
  referee_interfaces::msg::BasicHp basic_hp;
  int outpost_hp_limit = 0;
  int base_hp_limit = 0;

  if (!getInput("base_hp", basic_hp)) {
    RCLCPP_ERROR(logger_, "Failed to get basic_hp from blackboard");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("base_hp_limit", base_hp_limit)) {
    RCLCPP_ERROR(logger_, "Failed to get base_hp_limit from blackboard");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("outpost_hp_limit", outpost_hp_limit)) {
    RCLCPP_ERROR(logger_, "Failed to get outpost_hp_limit from blackboard");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(logger_, "[HomeCheck] 状态检查: 基地血量=%d, 前哨站血量=%d",
              basic_hp.base_hp, basic_hp.outpost_hp);

  if (basic_hp.base_hp <= base_hp_limit || basic_hp.outpost_hp <= outpost_hp_limit) {
    RCLCPP_WARN(logger_,
                "[HomeCheck] 状态不及预期:基地血量=%d, 前哨站血量=%d",
                basic_hp.base_hp, basic_hp.outpost_hp);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(logger_, "[HomeCheck] 状态良好: 基地血量=%d, 前哨站血量=%d",
              basic_hp.base_hp, basic_hp.outpost_hp);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList HomeCheckCondition::providedPorts() {
  return {
      BT::InputPort<referee_interfaces::msg::BasicHp>("basic_hp", "{@basic_hp}",
                                                      "哨兵基本状态信息"),
      BT::InputPort<int>("base_hp_limit", "{@base_hp_limit}", "基地血量阈值"),
      BT::InputPort<int>("outpost_hp_limit", "{@outpost_hp_limit}",
                         "前哨战血量阈值")};
}

} // namespace rm_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rm_behavior::HomeCheckCondition>("HomeCheck");
}
