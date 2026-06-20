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

#include "suave_bt/action_search_pipeline.hpp"

namespace suave_bt
{
SearchPipeline::SearchPipeline(const std::string & name, const BT::NodeConfig & conf)
: BtActionClient(name, conf, "spiral_search"), pipeline_detected_(false)
{
  if (!usesActionServer()) {
    pipeline_detection_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/pipeline/detected", 10,
      std::bind(&SearchPipeline::pipeline_detected_cb, this, std::placeholders::_1));
  }
}

void SearchPipeline::pipeline_detected_cb(const std_msgs::msg::Bool & msg)
{
  pipeline_detected_ = msg.data;
}

SearchPipeline::Action::Goal SearchPipeline::makeGoal() {return Action::Goal();}

BT::NodeStatus SearchPipeline::evaluateResult(const WrappedResult & result)
{
  if (
    result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result != nullptr &&
    result.result->pipeline_found)
  {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void SearchPipeline::onStartRequested() {node_->set_search_started();}

BT::NodeStatus SearchPipeline::onLegacyStart() {return BT::NodeStatus::RUNNING;}

BT::NodeStatus SearchPipeline::onLegacyRunning()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if (node_->time_limit_reached()) {
    std::cout << "Time limit reached. Canceling action " << this->name() << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  if (pipeline_detected_ == true) {
    std::cout << "Async action finished: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  std::cout << "Searching for pipeline! " << std::endl;
  return BT::NodeStatus::RUNNING;
}
}  // namespace suave_bt
