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

#include "suave_bt/action_change_mode.hpp"


namespace suave_bt
{
  using namespace std::placeholders;
  using namespace std::chrono_literals;

  ChangeMode::ChangeMode(
    const std::string& name, const BT::NodeConfig & conf)
  : BT::SyncActionNode(name, conf)
  {
    getInput("node_name", node_name_);

    node_ = config().blackboard->get<std::shared_ptr<suave_bt::SuaveMission>>("node");
    previous_modes_ = config().blackboard->get<std::shared_ptr<std::map<std::string, std::string>>>("previous_modes");

    change_mode_cli_ = node_->create_client<system_modes_msgs::srv::ChangeMode>(
      node_name_ + "/change_mode");
    get_mode_cli_ = node_->create_client<system_modes_msgs::srv::GetMode>(
      node_name_ + "/get_mode");
  }

  BT::NodeStatus ChangeMode::tick(){
    std::string mode_name;
    getInput("mode_name", mode_name);

    if(previous_modes_->at(node_name_) == mode_name) return BT::NodeStatus::SUCCESS;

    if(get_mode_cli_->service_is_ready()){
      auto request = std::make_shared<system_modes_msgs::srv::GetMode::Request>();
      auto get_mode_future_ = get_mode_cli_->async_send_request(request);

      if (get_mode_future_.wait_for(1s) == std::future_status::ready)
      {
        auto get_mode_result_ = get_mode_future_.get();
        if(get_mode_result_->current_mode == mode_name){
          return BT::NodeStatus::SUCCESS;
        }

        if(change_mode_cli_->service_is_ready()){
          auto request = std::make_shared<system_modes_msgs::srv::ChangeMode::Request>();
          request->mode_name = mode_name;
          auto change_mode_future_ = change_mode_cli_->async_send_request(request);

          if (change_mode_future_.wait_for(1s) == std::future_status::ready)
          {
            auto change_mode_result_ = change_mode_future_.get();
            if(!change_mode_result_->success){
              return BT::NodeStatus::FAILURE;
            }

            if(change_mode_result_->success){
              RCLCPP_INFO(node_->get_logger(), "Node %s mode changed to %s", node_name_.c_str(), mode_name.c_str());
              previous_modes_->at(node_name_) = mode_name;
              return BT::NodeStatus::SUCCESS;
            }
          } else {
            return BT::NodeStatus::FAILURE;
          }
        }

        RCLCPP_INFO(node_->get_logger(), "%s/change_mode service not available...", node_name_.c_str());
        return BT::NodeStatus::FAILURE;
      } else {
        return BT::NodeStatus::FAILURE;
      }
    }

    RCLCPP_INFO(node_->get_logger(), "%s/get_mode service not available...", node_name_.c_str());
    return BT::NodeStatus::FAILURE;

  }
} //namespace suave_bt
