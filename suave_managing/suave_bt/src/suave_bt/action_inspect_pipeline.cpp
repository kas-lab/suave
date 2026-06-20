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

#include "suave_bt/action_inspect_pipeline.hpp"

namespace suave_bt
{
InspectPipeline::InspectPipeline(const std::string & name, const BT::NodeConfig & conf)
: BtActionClient(name, conf, "follow_pipeline"), pipeline_inspected_(false)
{
  if (!usesActionServer()) {
    pipeline_inspected_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/pipeline/inspected", 10,
      std::bind(&InspectPipeline::pipeline_inspected_cb, this, std::placeholders::_1));
  }
}

void InspectPipeline::pipeline_inspected_cb(const std_msgs::msg::Bool & msg)
{
  pipeline_inspected_ = msg.data;
}

InspectPipeline::Action::Goal InspectPipeline::makeGoal()
{
  Action::Goal goal;
  goal.timeout = 0.0;
  return goal;
}

BT::NodeStatus InspectPipeline::evaluateResult(const WrappedResult & result)
{
  if (
    result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result != nullptr &&
    result.result->success)
  {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus InspectPipeline::onLegacyStart() {return BT::NodeStatus::RUNNING;}

BT::NodeStatus InspectPipeline::onLegacyRunning()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if (this->node_->time_limit_reached()) {
    std::cout << "Time limit reached. Canceling action " << this->name() << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  if (pipeline_inspected_ == true) {
    std::cout << "Async action finished: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  std::cout << "Inspecting pipeline! " << std::endl;
  return BT::NodeStatus::RUNNING;
}
}  // namespace suave_bt
