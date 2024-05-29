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

#include "suave_bt/suave_mission.hpp"

namespace suave_bt
{
  using namespace std::placeholders;
  using namespace std::chrono_literals;

  SuaveMission::SuaveMission(std::string none_name)
  : Node(none_name), search_started_(false), mission_aborted_(false)
  {
    this->declare_parameter("time_limit", 300);
    time_limit_ = this->get_parameter("time_limit").as_int();

    save_mission_results_cli =
      this->create_client<std_srvs::srv::Empty>("mission_metrics/save");

    time_limit_timer_cb_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    // TODO: create parameter for timer rate?
    time_limit_timer_ = this->create_wall_timer(
      100ms, std::bind(&SuaveMission::time_limit_cb, this), time_limit_timer_cb_group_);
  }

  void SuaveMission::time_limit_cb(){
    if(this->time_limit_reached()){
      RCLCPP_INFO(this->get_logger(), "Time limit reached!");
      this->finish_mission();
    }
  }

  bool SuaveMission::time_limit_reached(){
    if(search_started_){
      return (this->get_clock()->now() - start_time_) >= rclcpp::Duration(time_limit_, 0);
    }
    return false;
  }

  void SuaveMission::finish_mission(){
    this->request_save_mission_results();
    this->time_limit_timer_->cancel();
    mission_aborted_ = true;
  }

  bool SuaveMission::request_save_mission_results(){
    while (!save_mission_results_cli->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "mission_metrics/save service not available, waiting again...");
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto response = save_mission_results_cli->async_send_request(request);
    if (response.wait_for(1s) == std::future_status::ready)
    {
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service mission_metrics/save");
      return false;
    }
  }

  void SuaveMission::set_search_started(){
    if(search_started_ == true) return;
    start_time_ = this->get_clock()->now();
    search_started_ = true;
  }

}
