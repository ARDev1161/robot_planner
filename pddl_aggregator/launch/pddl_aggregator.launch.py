#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_pddl_topic = LaunchConfiguration('map_pddl_topic')
    people_topic = LaunchConfiguration('people_topic')
    objects_topic = LaunchConfiguration('objects_topic')
    robots_topic = LaunchConfiguration('robots_topic')
    problem_pddl_topic = LaunchConfiguration('problem_pddl_topic')
    domain_file = LaunchConfiguration('domain_file')
    problem_output_file = LaunchConfiguration('problem_output_file')
    problem_name = LaunchConfiguration('problem_name')
    domain_name = LaunchConfiguration('domain_name')
    people_predicate = LaunchConfiguration('people_predicate')
    objects_predicate = LaunchConfiguration('objects_predicate')
    robots_predicate = LaunchConfiguration('robots_predicate')

    return LaunchDescription([
        DeclareLaunchArgument('map_pddl_topic', default_value='/pddl/map'),
        DeclareLaunchArgument('people_topic', default_value=''),
        DeclareLaunchArgument('objects_topic', default_value=''),
        DeclareLaunchArgument('robots_topic', default_value=''),
        DeclareLaunchArgument('problem_pddl_topic', default_value='/pddl/problem'),
        DeclareLaunchArgument('domain_file', default_value='pddl/domain.pddl'),
        DeclareLaunchArgument('problem_output_file', default_value='/tmp/problem.pddl'),
        DeclareLaunchArgument('problem_name', default_value=''),
        DeclareLaunchArgument('domain_name', default_value=''),
        DeclareLaunchArgument('people_predicate', default_value='person_at'),
        DeclareLaunchArgument('objects_predicate', default_value=''),
        DeclareLaunchArgument('robots_predicate', default_value='at'),
        Node(
            package='pddl_aggregator',
            executable='pddl_aggregator',
            name='pddl_aggregator',
            output='screen',
            parameters=[{
                'map_pddl_topic': map_pddl_topic,
                'people_topic': people_topic,
                'objects_topic': objects_topic,
                'robots_topic': robots_topic,
                'problem_pddl_topic': problem_pddl_topic,
                'domain_file': domain_file,
                'problem_output_file': problem_output_file,
                'problem_name': problem_name,
                'domain_name': domain_name,
                'people_predicate': people_predicate,
                'objects_predicate': objects_predicate,
                'robots_predicate': robots_predicate,
            }],
        ),
    ])
