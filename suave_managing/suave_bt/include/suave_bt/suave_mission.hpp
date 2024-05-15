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

#ifndef SUAVE_BT__SUAVE_MISSION_HPP_
#define SUAVE_BT__SUAVE_MISSION_HPP_

#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/empty.hpp"

namespace suave_bt
{

class SuaveMission : public rclcpp::Node{

public:
  SuaveMission(std::string none_name);

  bool time_limit_reached();
  void set_search_started();
  bool is_mission_aborted(){return mission_aborted_;};
  void finish_mission();

private:
  rclcpp::Time start_time_;
  bool search_started_ = false;
  int time_limit_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr save_mission_results_cli;

  bool mission_aborted_;

  rclcpp::CallbackGroup::SharedPtr time_limit_timer_cb_group_;
  rclcpp::TimerBase::SharedPtr time_limit_timer_;

  void time_limit_cb();
  bool request_save_mission_results();
};

}  // namespace suave_bt

#endif  // SUAVE_BT__SUAVE_MISSION_HPP_
