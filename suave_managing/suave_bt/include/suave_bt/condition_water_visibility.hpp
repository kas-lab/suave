// Copyright 2023 Gustavo Rezende Silva
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

#ifndef SUAVE_BT__CONDITION_WATER_VISIBILITY_HPP_
#define SUAVE_BT__CONDITION_WATER_VISIBILITY_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "suave_bt/suave_mission.hpp"

using namespace std::chrono_literals;

namespace suave_bt
{

class WaterVisibility : public BT::ConditionNode
{
public:
  explicit WaterVisibility(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("threshold"),
      });
  }

private:
  std::shared_ptr<suave_bt::SuaveMission> node_;

  float water_visibility_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  void diagnostics_cb(const diagnostic_msgs::msg::DiagnosticArray &msg);
};

} //namespace suave_bt

#endif  // SUAVE_BT__CONDITION_WATER_VISIBILITY_HPP_
