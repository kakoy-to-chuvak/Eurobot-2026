# eurobot_bringup/launch/navigation.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Объявление аргументов командной строки ---
    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Full path to map YAML file'
    )
    
    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Full path to Nav2 params YAML file'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time'
    )
    
    # --- Получение значений аргументов ---
    map_file = LaunchConfiguration('map_file')
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Получаем директории пакетов
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # --- 1. Настройка map_server ---    
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time
        }]
    )
    
    # --- 2. Настройка lifecycle manager для карты ---
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[config_file],
    )
    
    # --- 3. Запуск самой навигации ---
    
    # Создаём словарь аргументов для Nav2
    nav2_launch_args = {
        'params_file': config_file,
        'use_sim_time': use_sim_time
    }
    
    nav2_bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch),
        launch_arguments=nav2_launch_args.items()
    )
    
    # --- 4. Сборка LaunchDescription ---
    ld = LaunchDescription([
        declare_map_file_arg,
        declare_config_file_arg,
        declare_use_sim_time_arg,
        map_server_cmd,
        lifecycle_manager_cmd,
        amcl_cmd,
        nav2_cmd
    ])
    
    return ld