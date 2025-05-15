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

#include "rm_behavior/plugins/condition/buff.hpp"

namespace rm_behavior
{
    BuffCondition::BuffCondition(
        const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
    }
    BT::NodeStatus BuffCondition::tick()
    {
        // 这里可以添加逻辑来检查buff的状态
        // 例如，检查buff是否处于激活状态
        bool buff_active = true;  // 假设buff处于激活状态

        if (buff_active) {
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }
    BT::PortsList BuffCondition::providedPorts()
    {
        return {
            BT::InputPort<bool>("buff_active", "Check if buff is active"),
        };
    }

}  // namespace rm_behavior
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE
{
  factory.registerNodeType<rm_behavior::BuffCondition>(
      "Buff");
}
// RM_BEHAVIOR__PLUGINS__CONDITION__BUFF_HPP_