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

#include "suave_bt/action_recharge_battery.hpp"

namespace suave_bt
{
  RechargeBattery::RechargeBattery(
    const std::string& name, const BT::NodeConfig & conf)
  : BT::StatefulActionNode(name, conf), recharged_(false)
  {
    node_ = config().blackboard->get<std::shared_ptr<suave_bt::SuaveMission>>("node");
    battery_level_sub_  = this->node_->create_subscription<std_msgs::msg::Bool>(
      "/battery_monitor/recharge/complete",
      10,
      std::bind(&RechargeBattery::battery_level_cb, this, _1));
  }

  void RechargeBattery::battery_level_cb(const std_msgs::msg::Bool &msg){
    recharged_ = msg.data;
  }

  BT::NodeStatus RechargeBattery::onStart() {
    recharged_ = false;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus RechargeBattery::onRunning() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if(this->node_->time_limit_reached()){
      std::cout << "Time limit reached. Canceling action "<< this->name() << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    if(recharged_ == true){
      std::cout << "Async action finished: "<< this->name() << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
    std::cout<<"Recharging! "<<std::endl;
    return BT::NodeStatus::RUNNING;
  }
} //namespace suave_bt
