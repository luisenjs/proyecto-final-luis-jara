#!/usr/bin/env python3
"""
Launch file para cargar el mapa y el planificador global.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Ruta al mapa desde el paquete go2_config
    go2_config_dir = get_package_share_directory('go2_config')
    default_map_path = os.path.join(go2_config_dir, 'maps', 'map.yaml')
    
    # Ruta al archivo de configuración de RViz
    planner_dir = get_package_share_directory('global_planner')
    rviz_config_file = os.path.join(planner_dir, 'rviz', 'planner.rviz')
    
    # Argumentos
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Ruta completa al archivo .yaml del mapa'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar tiempo de simulación'
    )
    
    # Nodo map_server (lifecycle)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': LaunchConfiguration('map')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Nodo lifecycle_manager para activar map_server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['map_server']},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Nodo planificador global Dijkstra
    planner_node = Node(
        package='global_planner',
        executable='global_planner_dijkstra',
        name='global_planner_dijkstra',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # RViz con configuración predefinida
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Transformación estática map -> odom (identidad)
    # Esto sincroniza el mapa con la odometría del robot
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    return LaunchDescription([
        declare_map_arg,
        declare_use_sim_time,
        map_server_node,
        lifecycle_manager_node,
        static_tf_node,
        planner_node,
        rviz_node
    ])
