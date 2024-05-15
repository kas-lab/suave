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

#include "suave_bt/is_pipeline_found.hpp"

namespace suave_bt
{

using namespace std::placeholders;

IsPipelineFound::IsPipelineFound(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(xml_tag_name, conf), _pipeline_detected(false)
{
  _node = config().blackboard->get<std::shared_ptr<suave_bt::SuaveMission>>("node");

  pipeline_detection_sub_  = _node->create_subscription<std_msgs::msg::Bool>(
    "/pipeline/detected",
    10,
    std::bind(&IsPipelineFound::pipeline_detected_cb, this, _1));
}

void
IsPipelineFound::pipeline_detected_cb(const std_msgs::msg::Bool &msg)
{
  _pipeline_detected = msg.data;
}

BT::NodeStatus
IsPipelineFound::tick()
{
  return (_pipeline_detected==true) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace suave_bt
