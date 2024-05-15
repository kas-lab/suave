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

#include "suave_bt/condition_thrusters_ok.hpp"

using namespace std::placeholders;

namespace suave_bt
{
  ThrustersOk::ThrustersOk(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf)
  : BT::ConditionNode(xml_tag_name, conf)
  {
    thrusters_ok_["c_thruster_1"] = true;
    thrusters_ok_["c_thruster_2"] = true;
    thrusters_ok_["c_thruster_3"] = true;
    thrusters_ok_["c_thruster_4"] = true;
    thrusters_ok_["c_thruster_5"] = true;
    thrusters_ok_["c_thruster_6"] = true;

    node_ = config().blackboard->get<std::shared_ptr<suave_bt::SuaveMission>>("node");
    diagnostics_sub_  = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics",
      10,
      std::bind(&ThrustersOk::diagnostics_cb, this, _1));
  }

  void ThrustersOk::diagnostics_cb(const diagnostic_msgs::msg::DiagnosticArray &msg){
    for(auto status: msg.status){
      if(status.message == "Component status"){
        for(auto value: status.values){
          auto it = thrusters_ok_.find(value.key);
          if (it != thrusters_ok_.end()) {
            it->second = (value.value == "RECOVERED") ? true : false;
          }
        }
      }
    }
  }


  BT::NodeStatus ThrustersOk::tick() {
    // check if whole thrusters_ok_ map is true
    if(std::all_of(thrusters_ok_.begin(), thrusters_ok_.end(), [](const auto& t) { return t.second; })){
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

}
