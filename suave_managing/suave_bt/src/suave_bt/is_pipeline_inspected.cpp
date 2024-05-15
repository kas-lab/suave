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

#include "suave_bt/is_pipeline_inspected.hpp"

namespace suave_bt
{

using namespace std::placeholders;

IsPipelineInspected::IsPipelineInspected(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(xml_tag_name, conf), _pipeline_inspected(false)
{
  _node = config().blackboard->get<std::shared_ptr<suave_bt::SuaveMission>>("node");

  pipeline_inspected_sub_  = _node->create_subscription<std_msgs::msg::Bool>(
    "/pipeline/inspected",
    10,
    std::bind(&IsPipelineInspected::pipeline_inspected_cb, this, _1));
}

void
IsPipelineInspected::pipeline_inspected_cb(const std_msgs::msg::Bool &msg)
{
  _pipeline_inspected = msg.data;
}

BT::NodeStatus
IsPipelineInspected::tick()
{
  return (_pipeline_inspected==true) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace suave_bt
