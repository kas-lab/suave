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

#ifndef SUAVE_BT__SET_GUIDED_MODE_HPP_
#define SUAVE_BT__SET_GUIDED_MODE_HPP_

#include <future>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

#include "suave_bt/suave_mission.hpp"

namespace suave_bt
{

class SetGuidedMode : public BT::StatefulActionNode{

public:
  SetGuidedMode(const std::string& name, const BT::NodeConfig & conf);

  BT::NodeStatus onStart() override {return BT::NodeStatus::RUNNING;};

  void onHalted() override {};

  BT::NodeStatus onRunning() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
      });
  }

protected:
  std::string mode_;
  suave_bt::SuaveMission::SharedPtr _node;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_guided_cli_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
  void state_cb(const mavros_msgs::msg::State &msg);
};

}  // namespace suave_bt

#endif  // SUAVE_BT__SET_GUIDED_MODE_HPP_
