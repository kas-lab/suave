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

#include "suave_bt/condition_water_visibility.hpp"

using namespace std::placeholders;

namespace suave_bt
{
  WaterVisibility::WaterVisibility(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf)
  : BT::ConditionNode(xml_tag_name, conf), water_visibility_(-1.0)
  {
    node_ = config().blackboard->get<std::shared_ptr<suave_bt::SuaveMission>>("node");
    diagnostics_sub_  = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics",
      10,
      std::bind(&WaterVisibility::diagnostics_cb, this, _1));
  }

  void WaterVisibility::diagnostics_cb(const diagnostic_msgs::msg::DiagnosticArray &msg){
    for(auto status: msg.status){
      if(status.message == "QA status"){
        for(auto value: status.values){
          if(value.key == "water_visibility") {
            water_visibility_ = std::stof(value.value);
          }
        }
      }
    }
  }

  BT::NodeStatus WaterVisibility::tick() {
    float threshold;
    getInput("threshold", threshold);
    if(water_visibility_ >= threshold || water_visibility_ == -1.0) return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
  }
}
