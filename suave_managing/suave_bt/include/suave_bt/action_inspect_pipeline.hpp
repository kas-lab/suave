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

#ifndef SUAVE_BT__INSPECT_PIPELINE_HPP_
#define SUAVE_BT__INSPECT_PIPELINE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "suave_bt/suave_mission.hpp"

using namespace std::placeholders;

namespace suave_bt
{

class InspectPipeline : public BT::StatefulActionNode{

public:
  InspectPipeline(const std::string& name, const BT::NodeConfig & conf);

  BT::NodeStatus onStart() override {return BT::NodeStatus::RUNNING;};

  BT::NodeStatus onRunning() override;

  void onHalted() override {};

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
      });
  }

private:
  std::shared_ptr<suave_bt::SuaveMission> node_;
  bool pipeline_inspected_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pipeline_inspected_sub_;
  void pipeline_inspected_cb(const std_msgs::msg::Bool &msg);
};

}  // namespace suave_bt

#endif  // SUAVE_BT__INSPECT_PIPELINE_HPP_
