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

# Добавляем исполняемые файлы из каталога src
add_executable(plan_executor src/plan_executor.cpp)
ament_target_dependencies(plan_executor rclcpp std_msgs yaml-cpp behaviortree_cpp_v3)

add_executable(bt_converter src/bt_converter.cpp)
ament_target_dependencies(bt_converter rclcpp std_msgs yaml-cpp behaviortree_cpp_v3)

# Установка исполняемых файлов
install(TARGETS
  plan_executor
  bt_converter
  DESTINATION lib/${PROJECT_NAME}
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
