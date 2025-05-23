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

//TODO 待测试,没有方式拿到 020B 数据

#include "rm_behavior/plugins/condition/game_status_check.hpp"

namespace rm_behavior
{

GameStatusCheckCondition::GameStatusCheckCondition(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config), logger_(rclcpp::get_logger("GameStatusCheck"))
{
}

BT::NodeStatus GameStatusCheckCondition::tick()
{
  int time_limit = 0;
  auto msg = getInput<referee_interfaces::msg::GameStatus>("game_status");
  
  if (!msg) {
    RCLCPP_ERROR(logger_, "Game status message is not available");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("time_limit", time_limit)) {
    RCLCPP_ERROR(logger_, "Failed to get time_limit from input");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(logger_, "Checking game state: %d vs time_limit: %d", 
               msg->game_progress, time_limit);

  // 检查比赛状态是否匹配
  // 4 比赛开始, 5 比赛结束
  if (msg->game_progress == time_limit) {
    RCLCPP_INFO(logger_, "[GameStatusCheck] 状态 %d 与目标状态匹配 -> SUCCESS", 
                 msg->game_progress);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(logger_, "[GameStatusCheck] 状态 %d 与目标状态不匹配 -> FAILURE", 
                 msg->game_progress);
    return BT::NodeStatus::FAILURE;
  }
}

BT::PortsList GameStatusCheckCondition::providedPorts()
{
  return {
    BT::InputPort<referee_interfaces::msg::GameStatus>("game_status", "{referee_gameStatus}", "裁判系统状态"),
    BT::InputPort<int>("time_limit", 4, "目标状态(相等时返回成功)")
  };
}

}  // namespace rm_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior::GameStatusCheckCondition>("GameStatusCheck");
}
