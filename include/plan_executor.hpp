#ifndef ROBOT_PLANNER__PLAN_EXECUTOR_HPP_
#define ROBOT_PLANNER__PLAN_EXECUTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <vector>
#include <map>
#include "common_types.hpp"

/// @file plan_executor.hpp
/// @brief Defines the PlanExecutor node which sequentially executes plans.

namespace robot_planner {

/// @brief ROS2 node that receives a plan and executes each action sequentially.
class PlanExecutor : public rclcpp::Node
{
public:
  /// Construct a PlanExecutor node
  explicit PlanExecutor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /// Default destructor
  ~PlanExecutor() override = default;

  /// Execute the given plan one action at a time
  /// @param plan Plan represented as a vector of actions
  void execute_plan(const std::vector<Action> & plan);

private:
  /// Callback processing an incoming plan in JSON form
  /// @param msg Message containing a JSON encoded plan
  void plan_callback(const std_msgs::msg::String::SharedPtr msg);

  /// Execute the "move" action
  /// @param params Parameters for the action
  void execute_move(const std::map<std::string, std::string> & params);
  /// Execute the "pick-up" action
  /// @param params Parameters for the action
  void execute_pickup(const std::map<std::string, std::string> & params);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr plan_subscription_;
};

}  // namespace robot_planner

#endif  // ROBOT_PLANNER__PLAN_EXECUTOR_HPP_
