#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

/// @file robot_planner_node.cpp
/// @brief Example node demonstrating integration with Nav2 and Plansys2.

class RobotPlannerNode : public rclcpp::Node
{
public:
  RobotPlannerNode() : Node("robot_planner_node")
  {
    // Инициализируем action-клиент для Nav2
    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Подписка на топик с командами от планировщика (симуляция)
    plan_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/plan_actions", 10,
      std::bind(&RobotPlannerNode::planCallback, this, std::placeholders::_1));

    // Инициализируем карту локаций (привязка имен к координатам)
    init_location_map();

    RCLCPP_INFO(this->get_logger(), "RobotPlannerNode запущена и ожидает команды.");
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr plan_sub_;
  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> location_map_;

  // Заполнение словаря локаций
  void init_location_map()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";

    // Пример для roomA
    pose.pose.position.x = 1.0;
    pose.pose.position.y = 1.0;
    pose.pose.orientation.w = 1.0;
    location_map_["roomA"] = pose;

    // Пример для roomB
    pose.pose.position.x = 2.0;
    pose.pose.position.y = 2.0;
    location_map_["roomB"] = pose;

    // Пример для roomC
    pose.pose.position.x = 3.0;
    pose.pose.position.y = 3.0;
    location_map_["roomC"] = pose;

    // Можно добавить и другие локации
  }

  // Callback для получения команд
  void planCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Получена команда: %s", msg->data.c_str());

    std::istringstream iss(msg->data);
    std::string action;
    iss >> action;

    if (action == "move") {
      std::string robot, from, to;
      iss >> robot >> from >> to;
      execute_move(from, to);
    } else if (action == "askcharge") {
      std::string robot, from, to;
      iss >> robot >> from >> to;
      execute_askcharge(from, to);
    } else if (action == "charge") {
      std::string robot, room;
      iss >> robot >> room;
      execute_charge(room);
    } else {
      RCLCPP_WARN(this->get_logger(), "Неизвестное действие: %s", action.c_str());
    }
  }

  // Реализация действия move
  void execute_move(const std::string & from, const std::string & to)
  {
    if (location_map_.find(to) == location_map_.end()) {
      RCLCPP_ERROR(this->get_logger(), "Локация %s не найдена", to.c_str());
      return;
    }

    if (!nav2_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action-сервер navigate_to_pose недоступен");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = location_map_[to];

    RCLCPP_INFO(this->get_logger(), "Отправляем цель для move: %s", to.c_str());
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      [this, to](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Действие move к %s выполнено успешно", to.c_str());
          // Здесь можно отправить обратную связь в Plansys2
        } else {
          RCLCPP_ERROR(this->get_logger(), "Ошибка выполнения move к %s", to.c_str());
        }
      };

    nav2_client_->async_send_goal(goal_msg, send_goal_options);
  }

  // Реализация действия askcharge
  void execute_askcharge(const std::string & from, const std::string & to)
  {
    // Для примера askcharge ведёт себя аналогично move – перемещает робота к зарядной станции.
    if (location_map_.find(to) == location_map_.end()) {
      RCLCPP_ERROR(this->get_logger(), "Локация %s не найдена", to.c_str());
      return;
    }

    if (!nav2_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action-сервер navigate_to_pose недоступен");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = location_map_[to];

    RCLCPP_INFO(this->get_logger(), "Отправляем цель для askcharge: %s", to.c_str());
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      [this, to](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Действие askcharge к %s выполнено успешно", to.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Ошибка выполнения askcharge к %s", to.c_str());
        }
      };

    nav2_client_->async_send_goal(goal_msg, send_goal_options);
  }

  // Реализация действия charge – симуляция зарядки (задержка и логирование)
  void execute_charge(const std::string & room)
  {
    if (location_map_.find(room) == location_map_.end()) {
      RCLCPP_ERROR(this->get_logger(), "Локация %s не найдена для зарядки", room.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Начинается зарядка в локации %s", room.c_str());
    // Симуляция зарядки длительностью 5 секунд
    rclcpp::sleep_for(5s);
    RCLCPP_INFO(this->get_logger(), "Зарядка завершена, батарея теперь full");
    // Здесь можно опубликовать сообщение об обновлении состояния батареи в систему
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
