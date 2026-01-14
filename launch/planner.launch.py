#!/usr/bin/env python3
"""
Launch file for the robot_planner package.
Запускает ноды планировщика, преобразователя плана и отдельные action-ноды:
move, pick-up, drop, visit, patrol_move, follow_person, search, wait, wait_and_watch.
Все параметры передаются из конфигурационного файла config.yaml.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _build_plansys2_include(context, *args, **kwargs):
    package_name = LaunchConfiguration('plansys2_launch_package').perform(context)
    launch_file = LaunchConfiguration('plansys2_launch_file').perform(context)
    domain_arg = LaunchConfiguration('plansys2_domain_arg').perform(context)
    problem_arg = LaunchConfiguration('plansys2_problem_arg').perform(context)
    domain_file = LaunchConfiguration('plansys2_domain_file').perform(context)
    problem_file = LaunchConfiguration('plansys2_problem_file').perform(context)
    use_sim_time = LaunchConfiguration('plansys2_use_sim_time').perform(context)
    namespace = LaunchConfiguration('plansys2_namespace').perform(context)

    launch_args = {
        domain_arg: domain_file,
        problem_arg: problem_file,
        'use_sim_time': use_sim_time,
    }
    if namespace:
        launch_args['namespace'] = namespace

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(package_name),
                    'launch',
                    launch_file,
                )
            ),
            launch_arguments=launch_args.items(),
        )
    ]

def generate_launch_description():
    # Определяем абсолютный путь к каталогу с конфигурацией
    config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
    config_file = os.path.join(config_dir, 'config.yaml')

    nodes = [
        DeclareLaunchArgument('map_pddl_topic', default_value='/pddl/map'),
        DeclareLaunchArgument('people_topic', default_value=''),
        DeclareLaunchArgument('objects_topic', default_value=''),
        DeclareLaunchArgument('robots_topic', default_value=''),
        DeclareLaunchArgument('problem_pddl_topic', default_value='/pddl/problem'),
        DeclareLaunchArgument('pddl_domain_file', default_value='pddl/domain.pddl'),
        DeclareLaunchArgument('pddl_problem_output_file', default_value='/tmp/problem.pddl'),
        DeclareLaunchArgument('plansys2_launch_package', default_value='plansys2_bringup'),
        DeclareLaunchArgument('plansys2_launch_file', default_value='plansys2_bringup_launch.py'),
        DeclareLaunchArgument('plansys2_domain_file', default_value='pddl/domain.pddl'),
        DeclareLaunchArgument('plansys2_problem_file', default_value='/tmp/problem.pddl'),
        DeclareLaunchArgument('plansys2_domain_arg', default_value='domain_file'),
        DeclareLaunchArgument('plansys2_problem_arg', default_value='problem_file'),
        DeclareLaunchArgument('plansys2_namespace', default_value=''),
        DeclareLaunchArgument('plansys2_use_sim_time', default_value='true'),
        DeclareLaunchArgument('plansys2_start_delay', default_value='2.0'),
        DeclareLaunchArgument('enable_plansys2', default_value='true'),
    ]

    # Единое приложение со всеми необходимыми нодами
    nodes.append(
        Node(
            package='robot_planner',
            executable='robot_planner',
            name='robot_planner',
            output='screen',
            parameters=[config_file, {'domain_template_topic': '/pddl_domain_template'}]
        )
    )

    nodes.append(
        Node(
            package='robot_planner',
            executable='pddl_aggregator',
            name='pddl_aggregator',
            output='screen',
            parameters=[config_file, {
                'map_pddl_topic': LaunchConfiguration('map_pddl_topic'),
                'people_topic': LaunchConfiguration('people_topic'),
                'objects_topic': LaunchConfiguration('objects_topic'),
                'robots_topic': LaunchConfiguration('robots_topic'),
                'problem_pddl_topic': LaunchConfiguration('problem_pddl_topic'),
                'domain_file': LaunchConfiguration('pddl_domain_file'),
                'problem_output_file': LaunchConfiguration('pddl_problem_output_file'),
            }]
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

    nodes.append(
        Node(
            package='robot_planner',
            executable='goal_gateway',
            name='goal_gateway',
            output='screen',
            parameters=[config_file]
        )
    )

    plansys2_group = GroupAction(
        actions=[
            TimerAction(
                period=LaunchConfiguration('plansys2_start_delay'),
                actions=[OpaqueFunction(function=_build_plansys2_include)],
            )
        ],
        condition=IfCondition(LaunchConfiguration('enable_plansys2')),
    )
    nodes.append(plansys2_group)

    return LaunchDescription(nodes)

if __name__ == '__main__':
    generate_launch_description()
