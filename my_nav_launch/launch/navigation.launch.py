# eurobot_bringup/launch/navigation.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1. Настройка map_server ---
    # Получаем путь к директории с вашим config-файлом.
    # Мы создадим папку 'config' позже, чтобы все параметры были в одном месте.
    config_file = os.path.join(get_package_share_directory('my_nav_launch'), 'nav_config', 'nav_param.yaml')
    map_file = os.path.join(get_package_share_directory('my_nav_launch'), 'map', 'map.yaml')
    
    # Узел сервера карт
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': False}]
    )

    # --- 2. Настройка lifecycle manager для карты---.
    # Он автоматически переведет map_server в состояние ACTIVE.
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # --- 3. Запуск самой навигации ---
    # Находим путь к стандартному launch-файлу Nav2 и запускаем его с нашими параметрами.
    nav2_bringup_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch),
        launch_arguments={
            'params_file': config_file,
            'use_sim_time': 'False'
        }.items()
    )

    # Собираем и возвращаем LaunchDescription
    ld = LaunchDescription([
        map_server_cmd,
        lifecycle_manager_cmd,
        nav2_cmd
    ])
    return ld