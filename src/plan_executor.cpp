#include "plan_executor.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include <nlohmann/json.hpp>
#include <iostream>

/// @file plan_executor.cpp
/// @brief Implementation of the PlanExecutor node.

namespace robot_planner {

using json = nlohmann::json;

PlanExecutor::PlanExecutor(const rclcpp::NodeOptions & options)
: Node("plan_executor_node", options)
{
  // Подписка на топик с сгенерированным планом (например, JSON-строка)
  plan_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "generated_plan", 10,
    std::bind(&PlanExecutor::plan_callback, this, std::placeholders::_1)
  );
  RCLCPP_INFO(this->get_logger(), "PlanExecutor node started.");
}



/**
 * @brief Parse a JSON plan received on the subscription and trigger execution.
 */
void PlanExecutor::plan_callback(const std_msgs::msg::String::SharedPtr msg)
{
  try {
    // Парсинг JSON-плана
    json plan_json = json::parse(msg->data);
    std::vector<Action> plan;
    for (const auto & action_item : plan_json) {
      Action action;
      action.name = action_item["name"];
      for (auto& param : action_item["params"].items()) {
        action.params[param.key()] = param.value();
      }
      plan.push_back(action);
    }
    RCLCPP_INFO(this->get_logger(), "Received plan with %zu actions", plan.size());
    execute_plan(plan);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse plan: %s", e.what());
  }
}

/**
 * @brief Execute all actions contained in the provided plan.
 */
void PlanExecutor::execute_plan(const std::vector<Action> & plan)
{
  for (const auto & action : plan) {
    RCLCPP_INFO(this->get_logger(), "Executing action: %s", action.name.c_str());
    if (action.name == "move") {
      execute_move(action.params);
    } else if (action.name == "pick-up") {
      execute_pickup(action.params);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown action: %s", action.name.c_str());
    }
  }
}

/**
 * @brief Handle the execution of a move action.
 */
void PlanExecutor::execute_move(const std::map<std::string, std::string> & params)
{
  auto it = params.find("target_room");
  if (it != params.end()) {
    RCLCPP_INFO(this->get_logger(), "Moving to room: %s", it->second.c_str());
    // Здесь можно реализовать вызов ROS-сервиса или action-сервера для перемещения робота.
  } else {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'target_room' not found for move action.");
  }
}

/**
 * @brief Handle the execution of a pick-up action.
 */
void PlanExecutor::execute_pickup(const std::map<std::string, std::string> & params)
{
  auto it = params.find("object");
  if (it != params.end()) {
    RCLCPP_INFO(this->get_logger(), "Picking up object: %s", it->second.c_str());
    // Здесь можно реализовать вызов ROS-сервиса или action-сервера для захвата объекта.
  } else {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'object' not found for pick-up action.");
  }
}

}  // namespace robot_planner
