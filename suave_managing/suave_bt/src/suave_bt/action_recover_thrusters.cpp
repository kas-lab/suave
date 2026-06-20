// Copyright 2026 KAS Lab
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

#include "suave_bt/action_recover_thrusters.hpp"

namespace suave_bt
{

RecoverThrusters::RecoverThrusters(const std::string & name, const BT::NodeConfig & config)
: BtActionClient(name, config, "recover_thrusters")
{
}

RecoverThrusters::Action::Goal RecoverThrusters::makeGoal() {return Action::Goal();}

BT::NodeStatus RecoverThrusters::evaluateResult(const WrappedResult & result)
{
  if (
    result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result != nullptr &&
    result.result->success)
  {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RecoverThrusters::onLegacyStart() {return BT::NodeStatus::SUCCESS;}

}  // namespace suave_bt
