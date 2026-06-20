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

#ifndef SUAVE_BT__ACTION_RECOVER_THRUSTERS_HPP_
#define SUAVE_BT__ACTION_RECOVER_THRUSTERS_HPP_

#include <string>

#include "suave_bt/bt_action_client.hpp"
#include "suave_msgs/action/recover_thrusters.hpp"

namespace suave_bt
{

class RecoverThrusters : public BtActionClient<suave_msgs::action::RecoverThrusters>
{
public:
  RecoverThrusters(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts() {return providedBasicPorts();}

protected:
  using Action = suave_msgs::action::RecoverThrusters;
  using WrappedResult = BtActionClient<Action>::WrappedResult;

  Action::Goal makeGoal() override;
  BT::NodeStatus evaluateResult(const WrappedResult & result) override;

private:
  BT::NodeStatus onLegacyStart() override;
};

}  // namespace suave_bt

#endif  // SUAVE_BT__ACTION_RECOVER_THRUSTERS_HPP_
