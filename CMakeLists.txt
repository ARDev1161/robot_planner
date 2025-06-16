cmake_minimum_required(VERSION 3.5)
project(robot_planner)

# Устанавливаем стандарт C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Поиск необходимых пакетов
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(include)

# Собираем единое приложение со всеми нодами
add_executable(robot_planner
  src/main.cpp
  src/plan_executor.cpp
  src/bt_converter.cpp
  src/pddl_template_receiver.cpp
)
ament_target_dependencies(robot_planner
  rclcpp
  std_msgs
  yaml-cpp
  behaviortree_cpp_v3
  plansys2_executor
  rclcpp_action
  nav2_msgs
)

# Action nodes
add_executable(ask_charge_action_node src/ask_charge_action_node.cpp)
ament_target_dependencies(ask_charge_action_node
  rclcpp
  plansys2_executor
  rclcpp_action
  nav2_msgs
  lifecycle_msgs
)

add_executable(charge_action_node src/charge_action_node.cpp)
ament_target_dependencies(charge_action_node
  rclcpp
  plansys2_executor
  rclcpp_action
  nav2_msgs
  lifecycle_msgs
)

add_executable(drop_action_node src/drop_action_node.cpp)
ament_target_dependencies(drop_action_node
  rclcpp
  plansys2_executor
  rclcpp_action
  nav2_msgs
  lifecycle_msgs
)

add_executable(follow_person_action_node src/follow_person_action_node.cpp)
ament_target_dependencies(follow_person_action_node
  rclcpp
  plansys2_executor
  rclcpp_action
  nav2_msgs
  lifecycle_msgs
)

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node
  rclcpp
  plansys2_executor
  rclcpp_action
  nav2_msgs
  lifecycle_msgs
)

add_executable(patrol_move_action_node src/patrol_move_action_node.cpp)
ament_target_dependencies(patrol_move_action_node
  rclcpp
  plansys2_executor
  rclcpp_action
  nav2_msgs
  lifecycle_msgs
)

add_executable(pickup_action_node src/pickup_action_node.cpp)
ament_target_dependencies(pickup_action_node
  rclcpp
  plansys2_executor
  rclcpp_action
  nav2_msgs
  lifecycle_msgs
)

add_executable(search_action_node src/search_action_node.cpp)
ament_target_dependencies(search_action_node
  rclcpp
  plansys2_executor
  rclcpp_action
  nav2_msgs
  lifecycle_msgs
)

add_executable(visit_action_node src/visit_action_node.cpp)
ament_target_dependencies(visit_action_node
  rclcpp
  plansys2_executor
  rclcpp_action
  nav2_msgs
  lifecycle_msgs
)

add_executable(wait_action_node src/wait_action_node.cpp)
ament_target_dependencies(wait_action_node
  rclcpp
  plansys2_executor
  rclcpp_action
  nav2_msgs
  lifecycle_msgs
)

add_executable(wait_and_watch_action_node src/wait_and_watch_action_node.cpp)
ament_target_dependencies(wait_and_watch_action_node
  rclcpp
  plansys2_executor
  rclcpp_action
  nav2_msgs
  lifecycle_msgs
)

# Установка исполняемых файлов
install(TARGETS
  robot_planner
  ask_charge_action_node
  charge_action_node
  drop_action_node
  follow_person_action_node
  move_action_node
  patrol_move_action_node
  pickup_action_node
  search_action_node
  visit_action_node
  wait_action_node
  wait_and_watch_action_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install public headers
install(DIRECTORY include/
  DESTINATION include
)

# Установка launch-файлов, конфигурации и PDDL файлов
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY pddl/
  DESTINATION share/${PROJECT_NAME}/pddl
)

ament_package()
