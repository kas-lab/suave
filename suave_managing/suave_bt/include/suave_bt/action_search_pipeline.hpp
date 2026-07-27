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

#ifndef SUAVE_BT__ACTION_SEARCH_PIPELINE_HPP_
#define SUAVE_BT__ACTION_SEARCH_PIPELINE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "suave_bt/bt_action_client.hpp"
#include "suave_msgs/action/spiral_search.hpp"

namespace suave_bt
{

class SearchPipeline : public BtActionClient<suave_msgs::action::SpiralSearch>
{
public:
  SearchPipeline(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts() {return {};}

protected:
  using Action = suave_msgs::action::SpiralSearch;
  using WrappedResult = BtActionClient<Action>::WrappedResult;

  Action::Goal makeGoal() override;
  BT::NodeStatus evaluateResult(const WrappedResult & result) override;

private:
  void onStartRequested() override;
  BT::NodeStatus onLegacyStart() override;
  BT::NodeStatus onLegacyRunning() override;

  bool pipeline_detected_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pipeline_detection_sub_;
  void pipeline_detected_cb(const std_msgs::msg::Bool & msg);
};

}  // namespace suave_bt

#endif  // SUAVE_BT__ACTION_SEARCH_PIPELINE_HPP_
