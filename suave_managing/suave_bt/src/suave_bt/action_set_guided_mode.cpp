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

#include "suave_bt/action_set_guided_mode.hpp"


namespace suave_bt
{
  using namespace std::placeholders;
  using namespace std::chrono_literals;

  SetGuidedMode::SetGuidedMode(
    const std::string& name, const BT::NodeConfig & conf)
  : BT::StatefulActionNode(name, conf)
  {
    _node = config().blackboard->get<std::shared_ptr<suave_bt::SuaveMission>>("node");
    set_guided_cli_ = _node->create_client<mavros_msgs::srv::SetMode>(
      "mavros/set_mode");
    mavros_state_sub_  = _node->create_subscription<mavros_msgs::msg::State>(
      "mavros/state",
      10,
      std::bind(&SetGuidedMode::state_cb, this, _1));
  }

  void
  SetGuidedMode::state_cb(const mavros_msgs::msg::State &msg)
  {
    mode_ = msg.mode;
  }


  BT::NodeStatus SetGuidedMode::onRunning(){
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if(set_guided_cli_->service_is_ready()){
      auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      request->custom_mode = "GUIDED";
      auto set_guided_result_ = set_guided_cli_->async_send_request(request);

      if (set_guided_result_.wait_for(1s) == std::future_status::ready)
      {
        auto result_ = set_guided_result_.get();
        if(!result_->mode_sent){
          return BT::NodeStatus::FAILURE;
        }

        if(result_->mode_sent && mode_ == "GUIDED"){
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mode set to GUIDED!");
          return BT::NodeStatus::SUCCESS;
        }
      } else {
        return BT::NodeStatus::FAILURE;
      }
    }

    return BT::NodeStatus::RUNNING;
  }
} //namespace suave_bt
