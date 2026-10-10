# -*- coding: utf-8 -*-
"""
地图定位（AMCL）+ 局部 EKF 里程计 + 轨迹显示

架构：
  - localization_launch.py（nav2）: map_server 发布 /map 占据栅格地图 + AMCL 发布 map -> odom
  - ekf_filter_node（robot_localization）: 融合 /odom + /imu，发布 odom -> base_footprint
  - path_recorder: 监听 map -> base_footprint，发布 /robot_trajectory 轨迹
  - rviz2: 显示地图 + AMCL 粒子云 + 机器人 + 轨迹 + EKF 里程计

运行方式（先启动仿真，再启动本文件）：
  ros2 launch fishbot_description gazebo_sim.launch.py
  ros2 launch fishbot_localization navigation_localization.launch.py
  ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 控制机器人，观察地图上的轨迹
"""
import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # ---------- 路径 ----------
    fishbot_localization_path = get_package_share_directory('fishbot_localization')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml_path = os.path.join(fishbot_localization_path, 'maps', 'room.yaml')
    nav2_param_path = os.path.join(fishbot_localization_path, 'config', 'nav2_params.yaml')
    ekf_param_path = os.path.join(fishbot_localization_path, 'config', 'ekf.yaml')
    rviz2_config_dir = os.path.join(
        fishbot_localization_path, 'config', 'rviz', 'navigation_ekf.rviz')

    # ---------- 启动参数 ----------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value=use_sim_time,
        description='Use simulation (Gazebo) clock if true')

    # ---------- 1. 地图 + AMCL 定位（map_server + amcl + lifecycle_manager）----------
    launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, '/launch', '/localization_launch.py']),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path,
            'autostart': 'true',
        }.items(),
    )

    # ---------- 2. 局部 EKF：发布 odom -> base_footprint 与 /odometry/filtered ----------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_param_path],
    )

    # ---------- 3. 轨迹记录：发布 /robot_trajectory ----------
    path_recorder_node = Node(
        package='fishbot_localization',
        executable='path_recorder',
        name='path_recorder',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---------- 4. RViz2 ----------
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return launch.LaunchDescription([
        declare_use_sim_time,
        launch_localization,
        ekf_node,
        path_recorder_node,
        rviz2_node,
    ])
