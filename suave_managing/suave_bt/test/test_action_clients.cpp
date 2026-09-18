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

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "behaviortree_cpp/bt_factory.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "system_modes_msgs/srv/change_mode.hpp"
#include "system_modes_msgs/srv/get_mode.hpp"
#include "suave_msgs/action/follow_pipeline.hpp"
#include "suave_msgs/action/recharge_battery.hpp"
#include "suave_msgs/action/recover_thrusters.hpp"
#include "suave_msgs/action/spiral_search.hpp"

#include "suave_bt/action_change_mode.hpp"
#include "suave_bt/action_inspect_pipeline.hpp"
#include "suave_bt/action_recharge_battery.hpp"
#include "suave_bt/action_recover_thrusters.hpp"
#include "suave_bt/action_search_pipeline.hpp"
#include "suave_bt/bt_action_client.hpp"
#include "suave_bt/suave_mission.hpp"

using namespace std::chrono_literals;

namespace
{

template<typename ActionT>
using ResultInitializer = std::function<void (typename ActionT::Result &)>;

template<typename ActionT>
typename rclcpp_action::Server<ActionT>::SharedPtr make_success_server(
  const rclcpp::Node::SharedPtr & node, const std::string & action_name,
  ResultInitializer<ActionT> initialize_result)
{
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  return rclcpp_action::create_server<ActionT>(
    node,
    action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<GoalHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [initialize_result](const std::shared_ptr<GoalHandle> goal_handle) {
      std::thread(
        [initialize_result, goal_handle]() {
          std::this_thread::sleep_for(10ms);
          auto result = std::make_shared<typename ActionT::Result>();
          initialize_result(*result);
          goal_handle->succeed(result);
        }).detach();
    });
}

template<typename BtNodeT, typename ActionT>
BT::NodeStatus run_action(
  const std::string & registration_name, const std::string & action_name,
  ResultInitializer<ActionT> initialize_result, bool use_action_server = true)
{
  static std::atomic<int> test_index{0};
  const auto suffix = std::to_string(test_index++);
  // Give the mock server a DDS-unique name so it never collides with a real
  // suave lifecycle-node action server of the same name running concurrently
  // in CI (colcon test runs packages in parallel on the same ROS domain).
  const std::string test_action_name = "test_" + action_name + "_" + suffix;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("use_action_server", use_action_server),
      rclcpp::Parameter(action_name + "_action_name", test_action_name)});
  auto mission = std::make_shared<suave_bt::SuaveMission>(
    "test_mission_node_" + suffix, options);
  auto server_node = std::make_shared<rclcpp::Node>(
    "test_action_server_" + suffix);
  auto server = make_success_server<ActionT>(
    server_node, test_action_name, initialize_result);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<BtNodeT>(registration_name);
  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::shared_ptr<suave_bt::SuaveMission>>("node", mission);
  blackboard->set<std::string>("test_action_name", action_name);
  const std::string xml =
    "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\">"
    "<BehaviorTree ID=\"Main\"><" + registration_name + "/>"
    "</BehaviorTree></root>";
  auto tree = factory.createTreeFromText(xml, blackboard);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mission);
  executor.add_node(server_node);
  std::thread spin_thread([&executor]() {executor.spin();});
  std::this_thread::sleep_for(200ms);

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  for (int attempt = 0; attempt < 100; ++attempt) {
    status = tree.rootNode()->executeTick();
    if (status != BT::NodeStatus::RUNNING) {
      break;
    }
    std::this_thread::sleep_for(10ms);
  }

  executor.cancel();
  spin_thread.join();
  (void)server;
  return status;
}

class TestRechargeClient
  : public suave_bt::BtActionClient<suave_msgs::action::RechargeBattery>
{
public:
  TestRechargeClient(const std::string & name, const BT::NodeConfig & config)
  : BtActionClient(
      name, config, config.blackboard->get<std::string>("test_action_name"))
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  using Action = suave_msgs::action::RechargeBattery;
  using WrappedResult = BtActionClient<Action>::WrappedResult;

  Action::Goal makeGoal() override {return Action::Goal();}

  BT::NodeStatus evaluateResult(const WrappedResult & result) override
  {
    return result.code == rclcpp_action::ResultCode::SUCCEEDED ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onLegacyStart() override {return BT::NodeStatus::SUCCESS;}
};

template<typename BtNodeT, typename ActionT>
class ResultEvaluator : public BtNodeT
{
public:
  using WrappedResult =
    typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;

  ResultEvaluator(const std::string & name, const BT::NodeConfig & config)
  : BtNodeT(name, config)
  {
  }

  BT::NodeStatus evaluate(const WrappedResult & result)
  {
    return this->evaluateResult(result);
  }
};

template<typename BtNodeT, typename ActionT>
BT::NodeStatus evaluate_result(ResultInitializer<ActionT> initialize_result)
{
  static std::atomic<int> evaluator_index{0};
  auto mission = std::make_shared<suave_bt::SuaveMission>(
    "result_evaluator_" + std::to_string(evaluator_index++));
  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::shared_ptr<suave_bt::SuaveMission>>("node", mission);
  BT::NodeConfig config;
  config.blackboard = blackboard;
  ResultEvaluator<BtNodeT, ActionT> evaluator("evaluator", config);

  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result;
  result.code = rclcpp_action::ResultCode::SUCCEEDED;
  result.result = std::make_shared<typename ActionT::Result>();
  initialize_result(*result.result);
  return evaluator.evaluate(result);
}

class ActionClientTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(ActionClientTest, search_maps_pipeline_found_to_success)
{
  const auto status =
    evaluate_result<suave_bt::SearchPipeline, suave_msgs::action::SpiralSearch>(
    [](auto & result) {result.pipeline_found = true;});
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(ActionClientTest, search_maps_pipeline_not_found_to_failure)
{
  const auto status =
    evaluate_result<suave_bt::SearchPipeline, suave_msgs::action::SpiralSearch>(
    [](auto & result) {result.pipeline_found = false;});
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(ActionClientTest, inspection_timeout_is_successful)
{
  const auto status =
    evaluate_result<suave_bt::InspectPipeline, suave_msgs::action::FollowPipeline>(
    [](auto & result) {
      result.success = true;
      result.timed_out = true;
    });
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(ActionClientTest, recharge_maps_unsuccessful_result_to_failure)
{
  const auto status =
    evaluate_result<suave_bt::RechargeBattery, suave_msgs::action::RechargeBattery>(
    [](auto & result) {result.success = false;});
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(ActionClientTest, recovery_maps_successful_result_to_success)
{
  const auto status =
    evaluate_result<suave_bt::RecoverThrusters, suave_msgs::action::RecoverThrusters>(
    [](auto & result) {result.success = true;});
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(ActionClientTest, legacy_mode_does_not_require_an_action_server)
{
  const auto status =
    run_action<TestRechargeClient, suave_msgs::action::RechargeBattery>(
    "test_action", "unused", [](auto &) {}, false);
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(ActionClientTest, rejected_goal_maps_to_failure)
{
  using Action = suave_msgs::action::RechargeBattery;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_action_server", true)});
  auto mission = std::make_shared<suave_bt::SuaveMission>(
    "rejection_test_mission", options);
  auto server_node = std::make_shared<rclcpp::Node>("rejection_test_server");
  auto server = rclcpp_action::create_server<Action>(
    server_node,
    "reject_recharge",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
      return rclcpp_action::GoalResponse::REJECT;
    },
    [](const std::shared_ptr<GoalHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](const std::shared_ptr<GoalHandle>) {});

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestRechargeClient>("test_action");
  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::shared_ptr<suave_bt::SuaveMission>>("node", mission);
  blackboard->set<std::string>("test_action_name", "reject_recharge");
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">"
    "<test_action/>"
    "</BehaviorTree></root>", blackboard);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mission);
  executor.add_node(server_node);
  std::thread spin_thread([&executor]() {executor.spin();});
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int attempt = 0; attempt < 100 && status == BT::NodeStatus::RUNNING; ++attempt) {
    status = tree.rootNode()->executeTick();
    std::this_thread::sleep_for(10ms);
  }
  executor.cancel();
  spin_thread.join();
  (void)server;
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(ActionClientTest, halt_cancels_an_accepted_goal)
{
  using Action = suave_msgs::action::RechargeBattery;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  std::atomic<bool> cancel_requested{false};
  std::shared_ptr<GoalHandle> active_goal;
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_action_server", true)});
  auto mission = std::make_shared<suave_bt::SuaveMission>(
    "cancel_test_mission", options);
  auto server_node = std::make_shared<rclcpp::Node>("cancel_test_server");
  auto server = rclcpp_action::create_server<Action>(
    server_node,
    "cancel_recharge",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [&cancel_requested](const std::shared_ptr<GoalHandle>) {
      cancel_requested = true;
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&active_goal](const std::shared_ptr<GoalHandle> goal_handle) {
      active_goal = goal_handle;
    });

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestRechargeClient>("test_action");
  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::shared_ptr<suave_bt::SuaveMission>>("node", mission);
  blackboard->set<std::string>("test_action_name", "cancel_recharge");
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">"
    "<test_action/>"
    "</BehaviorTree></root>", blackboard);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mission);
  executor.add_node(server_node);
  std::thread spin_thread([&executor]() {executor.spin();});
  bool remained_running = true;
  for (int attempt = 0; attempt < 50 && remained_running; ++attempt) {
    remained_running =
      tree.rootNode()->executeTick() == BT::NodeStatus::RUNNING;
    std::this_thread::sleep_for(10ms);
  }
  tree.haltTree();
  for (int attempt = 0; attempt < 100 && !cancel_requested; ++attempt) {
    std::this_thread::sleep_for(10ms);
  }
  executor.cancel();
  spin_thread.join();
  (void)server;
  EXPECT_TRUE(remained_running);
  EXPECT_TRUE(cancel_requested);
}

TEST_F(ActionClientTest, time_limit_stops_waiting_for_action_server)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("time_limit", 0),
      rclcpp::Parameter("use_action_server", true)});
  auto mission = std::make_shared<suave_bt::SuaveMission>(
    "time_limit_test_mission", options);
  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::shared_ptr<suave_bt::SuaveMission>>("node", mission);
  blackboard->set<std::string>("test_action_name", "missing_action_server");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestRechargeClient>("test_action");
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">"
    "<test_action/>"
    "</BehaviorTree></root>", blackboard);

  EXPECT_EQ(tree.rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  mission->set_search_started();
  EXPECT_EQ(tree.rootNode()->executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(ActionClientTest, inspection_maps_failure_result_to_failure)
{
  const auto status =
    evaluate_result<suave_bt::InspectPipeline, suave_msgs::action::FollowPipeline>(
    [](auto & result) {result.success = false;});
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(ActionClientTest, inspection_success_result_maps_to_success)
{
  const auto status =
    evaluate_result<suave_bt::InspectPipeline, suave_msgs::action::FollowPipeline>(
    [](auto & result) {
      result.success = true;
      result.timed_out = false;
    });
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(ActionClientTest, recovery_maps_failure_result_to_failure)
{
  const auto status =
    evaluate_result<suave_bt::RecoverThrusters, suave_msgs::action::RecoverThrusters>(
    [](auto & result) {result.success = false;});
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(ActionClientTest, search_action_mode_succeeds_when_pipeline_found)
{
  const auto status =
    run_action<suave_bt::SearchPipeline, suave_msgs::action::SpiralSearch>(
    "search_pipeline", "spiral_search",
    [](auto & result) {result.pipeline_found = true;});
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(ActionClientTest, search_action_mode_fails_when_pipeline_not_found)
{
  const auto status =
    run_action<suave_bt::SearchPipeline, suave_msgs::action::SpiralSearch>(
    "search_pipeline", "spiral_search",
    [](auto & result) {result.pipeline_found = false;});
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(ActionClientTest, inspect_action_mode_succeeds_when_action_succeeds)
{
  const auto status =
    run_action<suave_bt::InspectPipeline, suave_msgs::action::FollowPipeline>(
    "inspect_pipeline", "follow_pipeline",
    [](auto & result) {result.success = true;});
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(ActionClientTest, recover_action_mode_succeeds_when_action_succeeds)
{
  const auto status =
    run_action<suave_bt::RecoverThrusters, suave_msgs::action::RecoverThrusters>(
    "recover_thrusters", "recover_thrusters",
    [](auto & result) {result.success = true;});
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

// Drive a single <change_mode/> node against fake system_modes change_mode /
// get_mode, lifecycle get_state and get_parameters services. `lifecycle_state_id`
// is what the managed part node's get_state reports (PRIMARY_STATE_ACTIVE /
// _INACTIVE); pass 0 for "no get_state server". `reported_mode` is what get_mode
// returns (unused by the new code, kept to document intent). `spiral_altitude`
// is what the part node's get_parameters reports for the spiral sub-mode check.
// Pass provide_change_mode=false to omit that server.
BT::NodeStatus run_change_mode(
  const std::string & node_name, const std::string & mode_name,
  uint8_t lifecycle_state_id, const std::string & reported_mode = "",
  bool provide_change_mode = true, double spiral_altitude = 2.0)
{
  static std::atomic<int> change_mode_index{0};
  const auto suffix = std::to_string(change_mode_index++);

  auto mission = std::make_shared<suave_bt::SuaveMission>(
    "change_mode_mission_" + suffix);
  auto server_node = std::make_shared<rclcpp::Node>(
    "change_mode_servers_" + suffix);

  rclcpp::Service<system_modes_msgs::srv::ChangeMode>::SharedPtr change_mode_srv;
  if (provide_change_mode) {
    change_mode_srv = server_node->create_service<system_modes_msgs::srv::ChangeMode>(
      node_name + "/change_mode",
      [](const std::shared_ptr<system_modes_msgs::srv::ChangeMode::Request>,
      std::shared_ptr<system_modes_msgs::srv::ChangeMode::Response> response) {
        response->success = true;
      });
  }

  auto get_mode_srv = server_node->create_service<system_modes_msgs::srv::GetMode>(
    node_name + "/get_mode",
    [reported_mode](
      const std::shared_ptr<system_modes_msgs::srv::GetMode::Request>,
      std::shared_ptr<system_modes_msgs::srv::GetMode::Response> response) {
      response->current_mode = reported_mode;
    });

  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr get_state_srv;
  if (lifecycle_state_id != 0) {
    get_state_srv = server_node->create_service<lifecycle_msgs::srv::GetState>(
      node_name + "_node/get_state",
      [lifecycle_state_id](
        const std::shared_ptr<lifecycle_msgs::srv::GetState::Request>,
        std::shared_ptr<lifecycle_msgs::srv::GetState::Response> response) {
        response->current_state.id = lifecycle_state_id;
      });
  }

  auto get_params_srv =
    server_node->create_service<rcl_interfaces::srv::GetParameters>(
    node_name + "_node/get_parameters",
    [spiral_altitude](
      const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request>,
      std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response) {
      rcl_interfaces::msg::ParameterValue value;
      value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      value.double_value = spiral_altitude;
      response->values.push_back(value);
    });

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<suave_bt::ChangeMode>("change_mode");
  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::shared_ptr<suave_bt::SuaveMission>>("node", mission);
  auto previous_modes = std::make_shared<std::map<std::string, std::string>>();
  (*previous_modes)[node_name] = "";
  blackboard->set<std::shared_ptr<std::map<std::string, std::string>>>(
    "previous_modes", previous_modes);
  const std::string xml =
    "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\">"
    "<BehaviorTree ID=\"Main\">"
    "<change_mode node_name=\"" + node_name + "\" mode_name=\"" + mode_name + "\"/>"
    "</BehaviorTree></root>";
  auto tree = factory.createTreeFromText(xml, blackboard);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mission);
  executor.add_node(server_node);
  std::thread spin_thread([&executor]() {executor.spin();});
  std::this_thread::sleep_for(200ms);

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  for (int attempt = 0; attempt < 20; ++attempt) {
    status = tree.rootNode()->executeTick();
    if (status != BT::NodeStatus::RUNNING) {
      break;
    }
    std::this_thread::sleep_for(10ms);
  }

  executor.cancel();
  spin_thread.join();
  (void)change_mode_srv;
  (void)get_mode_srv;
  (void)get_state_srv;
  (void)get_params_srv;
  return status;
}

TEST_F(ActionClientTest, change_mode_succeeds_when_part_reaches_active)
{
  EXPECT_EQ(
    run_change_mode(
      "f_generate_search_path", "fd_spiral_medium",
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "fd_spiral_medium"),
    BT::NodeStatus::SUCCESS);
}

// Regression: system_modes reports the target mode (get_mode == mode_name, from
// a stale parameter default) while the part node never activated. The old code
// returned SUCCESS on the get_mode match; change_mode must now return FAILURE.
TEST_F(ActionClientTest, change_mode_fails_when_part_never_activates)
{
  EXPECT_EQ(
    run_change_mode(
      "f_generate_search_path", "fd_spiral_medium",
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "fd_spiral_medium"),
    BT::NodeStatus::FAILURE);
}

TEST_F(ActionClientTest, change_mode_unground_succeeds_when_part_inactive)
{
  EXPECT_EQ(
    run_change_mode(
      "f_generate_search_path", "fd_unground",
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "fd_spiral_medium"),
    BT::NodeStatus::SUCCESS);
}

TEST_F(ActionClientTest, change_mode_fails_when_change_mode_service_missing)
{
  EXPECT_EQ(
    run_change_mode(
      "f_generate_search_path", "fd_spiral_medium",
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "", false),
    BT::NodeStatus::FAILURE);
}

// "inactive" (used for generate_recharge_path in the extended tree) must expect
// the part node to be inactive, not active.
TEST_F(ActionClientTest, change_mode_inactive_succeeds_when_part_inactive)
{
  EXPECT_EQ(
    run_change_mode(
      "generate_recharge_path", "inactive",
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "normal"),
    BT::NodeStatus::SUCCESS);
}

TEST_F(ActionClientTest, change_mode_succeeds_when_spiral_altitude_matches)
{
  EXPECT_EQ(
    run_change_mode(
      "f_generate_search_path", "fd_spiral_high",
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "fd_spiral_high",
      true, 3.0),
    BT::NodeStatus::SUCCESS);
}

// Sub-mode gap: the part node is active but stuck at the previous altitude
// (2.0 == fd_spiral_medium) instead of the requested fd_spiral_high (3.0).
TEST_F(ActionClientTest, change_mode_fails_when_spiral_altitude_stale)
{
  EXPECT_EQ(
    run_change_mode(
      "f_generate_search_path", "fd_spiral_high",
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "fd_spiral_high",
      true, 2.0),
    BT::NodeStatus::FAILURE);
}

TEST_F(ActionClientTest, search_pipeline_calls_set_search_started_on_action_start)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("time_limit", 0),
      rclcpp::Parameter("use_action_server", true)});
  auto mission = std::make_shared<suave_bt::SuaveMission>(
    "search_started_test_mission", options);
  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::shared_ptr<suave_bt::SuaveMission>>("node", mission);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<suave_bt::SearchPipeline>("search_pipeline");
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">"
    "<search_pipeline/>"
    "</BehaviorTree></root>", blackboard);

  // onStart fires onStartRequested which calls set_search_started
  EXPECT_EQ(tree.rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  // time_limit_reached() is now true, so onRunning returns FAILURE
  EXPECT_EQ(tree.rootNode()->executeTick(), BT::NodeStatus::FAILURE);
}

}  // namespace
