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

#ifndef SUAVE_BT__ARM_THRUSTERS_HPP_
#define SUAVE_BT__ARM_THRUSTERS_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "suave_bt/suave_mission.hpp"


namespace suave_bt
{

class ArmThrusters : public BT::StatefulActionNode{

public:
  ArmThrusters(const std::string& name, const BT::NodeConfig & conf);

  BT::NodeStatus onRunning() override;

  BT::NodeStatus onStart() override {return BT::NodeStatus::RUNNING;};

  void onHalted() override {};

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
      });
  }

protected:
  suave_bt::SuaveMission::SharedPtr node_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_motors_cli_;
};

}  // namespace suave_bt

#endif  // SUAVE_BT__ARM_THRUSTERS_HPP_
