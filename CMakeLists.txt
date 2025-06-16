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
find_package(launch_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

include_directories(include)

# Собираем единое приложение со всеми нодами
add_executable(robot_planner
  src/main.cpp
  src/plan_executor.cpp
  src/bt_converter.cpp
  src/pddl_template_receiver.cpp
)
ament_target_dependencies(robot_planner rclcpp std_msgs yaml-cpp behaviortree_cpp_v3)

# Установка исполняемых файлов
install(TARGETS
  robot_planner
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
