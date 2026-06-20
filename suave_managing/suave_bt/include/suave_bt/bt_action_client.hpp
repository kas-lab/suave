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

#ifndef SUAVE_BT__BT_ACTION_CLIENT_HPP_
#define SUAVE_BT__BT_ACTION_CLIENT_HPP_

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "suave_bt/suave_mission.hpp"

namespace suave_bt
{

template<typename ActionT>
class BtActionClient : public BT::StatefulActionNode
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;

  BtActionClient(
    const std::string & name, const BT::NodeConfig & config, const std::string & action_name)
  : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<std::shared_ptr<SuaveMission>>("node");
    use_action_server_ = config.blackboard->get<bool>("use_action_server");
    if (use_action_server_) {
      action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name);
    }
  }

  static BT::PortsList providedBasicPorts()
  {
    return {BT::InputPort<bool>(
        "call_action_server", false, "Send a goal instead of observing legacy lifecycle behavior")};
  }

  BT::NodeStatus onStart() override
  {
    if (!getInput("call_action_server", call_action_server_)) {
      throw BT::RuntimeError("missing required input [call_action_server]");
    }
    if (call_action_server_ != use_action_server_) {
      throw BT::RuntimeError(
              "input [call_action_server] does not match blackboard "
              "[use_action_server]");
    }

    onStartRequested();
    if (!call_action_server_) {
      return onLegacyStart();
    }

    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      goal_handle_.reset();
      result_.reset();
      goal_rejected_ = false;
      halt_requested_ = false;
      waiting_for_server_ = false;
      ++generation_;
    }

    if (!action_client_->action_server_is_ready()) {
      std::lock_guard<std::mutex> lock(action_mutex_);
      waiting_for_server_ = true;
      return BT::NodeStatus::RUNNING;
    }

    sendGoal();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!call_action_server_) {
      return onLegacyRunning();
    }

    if (node_->time_limit_reached()) {
      RCLCPP_INFO(node_->get_logger(), "Time limit reached. Canceling action %s", name().c_str());
      requestGoalCancellation();
      return BT::NodeStatus::FAILURE;
    }

    bool waiting_for_server;
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      waiting_for_server = waiting_for_server_;
    }
    if (waiting_for_server) {
      if (action_client_->action_server_is_ready()) {
        {
          std::lock_guard<std::mutex> lock(action_mutex_);
          waiting_for_server_ = false;
        }
        sendGoal();
      }
      return BT::NodeStatus::RUNNING;
    }

    std::optional<WrappedResult> result;
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      if (goal_rejected_) {
        return BT::NodeStatus::FAILURE;
      }
      result = result_;
    }

    if (!result.has_value()) {
      return BT::NodeStatus::RUNNING;
    }
    return evaluateResult(result.value());
  }

  void onHalted() override
  {
    if (!call_action_server_) {
      onLegacyHalted();
      return;
    }

    requestGoalCancellation();
  }

protected:
  virtual typename ActionT::Goal makeGoal() = 0;
  virtual BT::NodeStatus evaluateResult(const WrappedResult & result) = 0;
  virtual void onStartRequested() {}
  virtual BT::NodeStatus onLegacyStart() {return BT::NodeStatus::RUNNING;}
  virtual BT::NodeStatus onLegacyRunning() {return BT::NodeStatus::RUNNING;}
  virtual void onLegacyHalted() {}

  bool usesActionServer() const {return use_action_server_;}

  std::shared_ptr<SuaveMission> node_;

private:
  void requestGoalCancellation()
  {
    typename GoalHandle::SharedPtr goal_to_cancel;
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      halt_requested_ = true;
      waiting_for_server_ = false;
      goal_to_cancel = goal_handle_;
    }
    if (goal_to_cancel != nullptr) {
      action_client_->async_cancel_goal(goal_to_cancel);
    }
  }

  void sendGoal()
  {
    std::size_t generation;
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      generation = generation_;
    }
    typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
    options.goal_response_callback = [this,
        generation](typename GoalHandle::SharedPtr goal_handle) {
        typename GoalHandle::SharedPtr goal_to_cancel;
        {
          std::lock_guard<std::mutex> lock(action_mutex_);
          if (generation != generation_) {
            goal_to_cancel = goal_handle;
          } else {
            goal_handle_ = goal_handle;
            goal_rejected_ = goal_handle_ == nullptr;
            if (halt_requested_) {
              goal_to_cancel = goal_handle_;
            }
          }
        }
        if (goal_to_cancel != nullptr) {
          action_client_->async_cancel_goal(goal_to_cancel);
        }
      };
    options.result_callback = [this, generation](const WrappedResult & result) {
        std::lock_guard<std::mutex> lock(action_mutex_);
        if (generation == generation_) {
          result_ = result;
        }
      };

    auto goal_future = action_client_->async_send_goal(makeGoal(), options);
    std::lock_guard<std::mutex> lock(action_mutex_);
    goal_futures_.erase(
      std::remove_if(
        goal_futures_.begin(), goal_futures_.end(),
        [](const auto & future) {
          return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
        }),
      goal_futures_.end());
    goal_futures_.push_back(goal_future);
  }

  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_;
  std::vector<std::shared_future<typename GoalHandle::SharedPtr>> goal_futures_;
  typename GoalHandle::SharedPtr goal_handle_;
  std::optional<WrappedResult> result_;
  std::mutex action_mutex_;
  bool call_action_server_ = false;
  bool use_action_server_ = false;
  bool goal_rejected_ = false;
  bool halt_requested_ = false;
  bool waiting_for_server_ = false;
  std::size_t generation_ = 0;
};

}  // namespace suave_bt

#endif  // SUAVE_BT__BT_ACTION_CLIENT_HPP_
