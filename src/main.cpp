#include "rclcpp/rclcpp.hpp"
#include "plan_executor.hpp"
#include "bt_converter.hpp"
#include "pddl_template_receiver.hpp"
#include <vector>
#include <memory>

/// @file main.cpp
/// @brief Example entry point demonstrating usage of PlanExecutor and BTConverter.

int main(int argc, char ** argv)
{
  // Инициализация ROS2
  rclcpp::init(argc, argv);

  // Создаем ноды планировщика и приема PDDL шаблонов
  auto executor_node = std::make_shared<robot_planner::PlanExecutor>();
  auto template_receiver = std::make_shared<robot_planner::PddlTemplateReceiver>();

  // Для демонстрации создадим статический план
  std::vector<robot_planner::Action> plan;
  plan.push_back({"move", {{"target_room", "room_10"}}});
  plan.push_back({"pick-up", {{"object", "target1"}}});

  // Преобразуем план в XML для поведенческого дерева с помощью BTConverter
  robot_planner::BTConverter bt_converter;
  std::string bt_xml = bt_converter.convertPlanToXML(plan);
  RCLCPP_INFO(executor_node->get_logger(), "Generated BT XML:\n%s", bt_xml.c_str());

  // Запускаем обе ноды в многопоточном исполнителье
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(executor_node);
  exec.add_node(template_receiver);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
