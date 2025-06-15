#!/usr/bin/env python3
"""
Launch file for the robot_planner package.
Запускает ноды планировщика, преобразователя плана и отдельные action-ноды:
move, pick-up, drop, visit, patrol_move, follow_person, search, wait, wait_and_watch.
Все параметры передаются из конфигурационного файла config.yaml.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Определяем абсолютный путь к каталогу с конфигурацией
    config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
    config_file = os.path.join(config_dir, 'config.yaml')

    nodes = []

    # Планировочная нода, которая обрабатывает поступающий план (например, из топика)
    nodes.append(
        Node(
            package='robot_planner',
            executable='plan_executor',
            name='plan_executor_node',
            output='screen',
            parameters=[config_file]
        )
    )

    # Нода для преобразования плана в поведенческое дерево (BT)
    nodes.append(
        Node(
            package='robot_planner',
            executable='bt_converter',
            name='bt_converter_node',
            output='screen',
            parameters=[config_file]
        )
    )

    # Нода для получения PDDL домена из шаблона
    nodes.append(
        Node(
            package='robot_planner',
            executable='pddl_template_receiver',
            name='pddl_template_receiver',
            output='screen',
            parameters=[config_file, {'domain_template_topic': '/pddl_domain_template'}]
        )
    )

    # Отдельные action-ноды для каждого действия, описанного в PDDL
    nodes.append(
        Node(
            package='robot_planner',
            executable='move_action_node',
            name='move_action_node',
            output='screen',
            parameters=[config_file]
        )
    )

    nodes.append(
        Node(
            package='robot_planner',
            executable='pickup_action_node',
            name='pickup_action_node',
            output='screen',
            parameters=[config_file]
        )
    )

    nodes.append(
        Node(
            package='robot_planner',
            executable='drop_action_node',
            name='drop_action_node',
            output='screen',
            parameters=[config_file]
        )
    )

    nodes.append(
        Node(
            package='robot_planner',
            executable='visit_action_node',
            name='visit_action_node',
            output='screen',
            parameters=[config_file]
        )
    )

    nodes.append(
        Node(
            package='robot_planner',
            executable='patrol_move_action_node',
            name='patrol_move_action_node',
            output='screen',
            parameters=[config_file]
        )
    )

    nodes.append(
        Node(
            package='robot_planner',
            executable='follow_person_action_node',
            name='follow_person_action_node',
            output='screen',
            parameters=[config_file]
        )
    )

    nodes.append(
        Node(
            package='robot_planner',
            executable='search_action_node',
            name='search_action_node',
            output='screen',
            parameters=[config_file]
        )
    )

    nodes.append(
        Node(
            package='robot_planner',
            executable='wait_action_node',
            name='wait_action_node',
            output='screen',
            parameters=[config_file]
        )
    )

    nodes.append(
        Node(
            package='robot_planner',
            executable='wait_and_watch_action_node',
            name='wait_and_watch_action_node',
            output='screen',
            parameters=[config_file]
        )
    )

    return LaunchDescription(nodes)

if __name__ == '__main__':
    generate_launch_description()
