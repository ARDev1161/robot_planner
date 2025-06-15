#include "rclcpp/rclcpp.hpp"
#include "robot_planner/plan_executor.hpp"
#include "robot_planner/bt_converter.hpp"
#include <vector>

int main(int argc, char ** argv)
{
  // Инициализация ROS2
  rclcpp::init(argc, argv);

  // Создаем ноду планировщика
  auto executor_node = std::make_shared<robot_planner::PlanExecutor>();

  // Для демонстрации создадим статический план
  std::vector<robot_planner::Action> plan;
  plan.push_back({"move", {{"target_room", "room_10"}}});
  plan.push_back({"pick-up", {{"object", "target1"}}});

  // Преобразуем план в XML для поведенческого дерева с помощью BTConverter
  robot_planner::BTConverter bt_converter;
  std::string bt_xml = bt_converter.convertPlanToXML(plan);
  RCLCPP_INFO(executor_node->get_logger(), "Generated BT XML:\n%s", bt_xml.c_str());

  // Спиним ноду для обработки входящих сообщений (например, для планов, поступающих по топику)
  rclcpp::spin(executor_node);
  rclcpp::shutdown();
  return 0;
}
