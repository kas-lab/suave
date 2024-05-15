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

#ifndef SUAVE_BT__CHANGE_MODE_HPP_
#define SUAVE_BT__CHANGE_MODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "suave_bt/suave_mission.hpp"
#include "system_modes_msgs/srv/change_mode.hpp"
#include "system_modes_msgs/srv/get_mode.hpp"


namespace suave_bt
{

class ChangeMode : public BT::SyncActionNode{

public:
  ChangeMode(const std::string& name, const BT::NodeConfig & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("node_name"),
        BT::InputPort<std::string>("mode_name"),
      });
  }

protected:
  suave_bt::SuaveMission::SharedPtr node_;
  std::string node_name_;
  std::shared_ptr<std::map<std::string, std::string>> previous_modes_;
  rclcpp::Client<system_modes_msgs::srv::ChangeMode>::SharedPtr change_mode_cli_;
  rclcpp::Client<system_modes_msgs::srv::GetMode>::SharedPtr get_mode_cli_;
};

}  // namespace suave_bt

#endif  // SUAVE_BT__CHANGE_MODE_HPP_
