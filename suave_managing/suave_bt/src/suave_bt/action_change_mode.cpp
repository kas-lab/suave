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

#include <cmath>

#include "suave_bt/action_change_mode.hpp"

namespace suave_bt
{
using namespace std::placeholders;
using namespace std::chrono_literals;

ChangeMode::ChangeMode(const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
  getInput("node_name", node_name_);

  node_ = config().blackboard->get<std::shared_ptr<suave_bt::SuaveMission>>("node");
  previous_modes_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, std::string>>>("previous_modes");

  change_mode_cli_ =
    node_->create_client<system_modes_msgs::srv::ChangeMode>(node_name_ + "/change_mode");
  get_state_cli_ =
    node_->create_client<lifecycle_msgs::srv::GetState>(node_name_ + "_node/get_state");
  get_params_cli_ =
    node_->create_client<rcl_interfaces::srv::GetParameters>(
    node_name_ + "_node/get_parameters");
}

bool ChangeMode::mode_implies_active(const std::string & mode_name)
{
  // See suave_modes.yaml: fd_unground and inactive map every system's part to
  // inactive, fd_all_thrusters maps f_maintain_motion's part to inactive.
  return mode_name != "fd_unground" &&
         mode_name != "fd_all_thrusters" &&
         mode_name != "inactive";
}

bool ChangeMode::part_in_expected_state(const std::string & mode_name)
{
  const bool want_active = mode_implies_active(mode_name);

  if (!get_state_cli_->service_is_ready()) {
    return false;
  }
  auto state_future = get_state_cli_->async_send_request(
    std::make_shared<lifecycle_msgs::srv::GetState::Request>());
  if (state_future.wait_for(1s) != std::future_status::ready) {
    return false;
  }
  auto state_id = state_future.get()->current_state.id;
  const bool state_ok = want_active ?
    state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE :
    state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  if (!state_ok) {
    return false;
  }

  // Sub-mode check: only f_generate_search_path selects a parameter
  // (spiral_altitude) per mode. For every other system/mode the lifecycle
  // state is the whole story.
  double expected_altitude = 0.0;
  if (mode_name == "fd_spiral_high") {
    expected_altitude = 3.0;
  } else if (mode_name == "fd_spiral_medium") {
    expected_altitude = 2.0;
  } else if (mode_name == "fd_spiral_low") {
    expected_altitude = 1.0;
  } else {
    return true;
  }

  if (!get_params_cli_->service_is_ready()) {
    return false;
  }
  auto params_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  params_request->names.push_back("spiral_altitude");
  auto params_future = get_params_cli_->async_send_request(params_request);
  if (params_future.wait_for(1s) != std::future_status::ready) {
    return false;
  }
  auto params_response = params_future.get();
  const auto & values = params_response->values;
  if (values.empty() ||
    values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
  {
    return false;
  }
  return std::abs(values[0].double_value - expected_altitude) < 0.01;
}

BT::NodeStatus ChangeMode::tick()
{
  std::string mode_name;
  getInput("mode_name", mode_name);

  // Already driven to this mode and the part node is where the mode requires
  // it to be: nothing to do.
  if (previous_modes_->at(node_name_) == mode_name && part_in_expected_state(mode_name)) {
    return BT::NodeStatus::SUCCESS;
  }

  if (!change_mode_cli_->service_is_ready()) {
    RCLCPP_INFO(
      node_->get_logger(), "%s/change_mode service not available...", node_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<system_modes_msgs::srv::ChangeMode::Request>();
  request->mode_name = mode_name;
  auto change_mode_future = change_mode_cli_->async_send_request(request);
  if (change_mode_future.wait_for(1s) != std::future_status::ready ||
    !change_mode_future.get()->success)
  {
    return BT::NodeStatus::FAILURE;
  }

  // system_modes acknowledges the mode change before the underlying lifecycle
  // transition and parameter update are applied (and can drop them entirely).
  // Only report success once the part node has actually reached the required
  // lifecycle state and spiral_altitude; otherwise fail so the tree keeps
  // re-driving the transition.
  if (!part_in_expected_state(mode_name)) {
    RCLCPP_INFO(
      node_->get_logger(),
      "%s: change_mode(%s) acknowledged but %s_node has not reached it yet",
      node_name_.c_str(), mode_name.c_str(), node_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Node %s mode changed to %s", node_name_.c_str(),
    mode_name.c_str());
  previous_modes_->at(node_name_) = mode_name;
  return BT::NodeStatus::SUCCESS;
}
}  // namespace suave_bt
