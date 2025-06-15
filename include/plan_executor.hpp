#ifndef ROBOT_PLANNER__PLAN_EXECUTOR_HPP_
#define ROBOT_PLANNER__PLAN_EXECUTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <map>

// Определяем структуру действия, которая будет использоваться в планах
namespace robot_planner {

struct Action {
  std::string name;
  std::map<std::string, std::string> params;
};

class PlanExecutor : public rclcpp::Node
{
public:
  explicit PlanExecutor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~PlanExecutor();

  // Функция для последовательного выполнения плана
  void execute_plan(const std::vector<Action> & plan);

private:
  // Callback для обработки входящего сообщения с планом (в формате JSON)
  void plan_callback(const std_msgs::msg::String::SharedPtr msg);

  // Функции для выполнения конкретных действий
  void execute_move(const std::map<std::string, std::string> & params);
  void execute_pickup(const std::map<std::string, std::string> & params);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr plan_subscription_;
};

}  // namespace robot_planner

#endif  // ROBOT_PLANNER__PLAN_EXECUTOR_HPP_
