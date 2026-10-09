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

# ============================================================================
# 下面是原来 fishbot_navigation2/launch/navigation2.launch.py 的代码，
# 按要求保留并注释掉作为参考。
# ============================================================================
# import launch
# import launch_ros
# from ament_index_python.packages import get_package_share_directory
# import os
# from launch.substitutions import LaunchConfiguration
# from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
#
# #ros2 launch fishbot_navigation2 navigation2.launch.py
#
# def generate_launch_description():
#     fishbot_navigation_path=get_package_share_directory("fishbot_navigation2")
#     nav2_bringup_dir=get_package_share_directory("nav2_bringup")
#     rviz2_config_dir=os.path.join(nav2_bringup_dir,"rviz","nav2_default_view.rviz")
#
#     use_sim_time=LaunchConfiguration("use_sim_time",default="true")
#     map_yaml_path=LaunchConfiguration("map",default=os.path.join(fishbot_navigation_path,"maps","room.yaml"))
#     nav2_param_path=LaunchConfiguration("params_file",default=os.path.join(fishbot_navigation_path,"config","nav2_params.yaml"))
#
#     use_sim_time_description=DeclareLaunchArgument(
#         name="use_sim_time", default_value=use_sim_time,
#         description="Use simulation(Gazebo) clock if true")
#     map_yaml_path_description=DeclareLaunchArgument(
#         name="map", default_value=map_yaml_path,
#         description="Full path to map file to load")
#     nav2_param_path_description=DeclareLaunchArgument(
#         name="params_file", default_value=nav2_param_path,
#         description="Full path to param file to load")
#
#     launch_navigation2=IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             [nav2_bringup_dir,"/launch","/bringup_launch.py"]),
#         launch_arguments={
#             "map":map_yaml_path,
#             "use_sim_time":use_sim_time,
#             "params_file":nav2_param_path
#         }.items())
#
#     rviz2_node=Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         arguments=["-d",rviz2_config_dir],
#         parameters=[{"use_sim_time":use_sim_time}],
#         output="screen")
#
#     return launch.LaunchDescription([
#         use_sim_time_description,
#         map_yaml_path_description,
#         nav2_param_path_description,
#         launch_navigation2,
#         rviz2_node])
# ============================================================================


def generate_launch_description():
    # ---------- 路径 ----------
    fishbot_navigation_path = get_package_share_directory('fishbot_navigation2')
    fishbot_localization_path = get_package_share_directory('fishbot_localization')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml_path = os.path.join(fishbot_navigation_path, 'maps', 'room.yaml')
    nav2_param_path = os.path.join(fishbot_navigation_path, 'config', 'nav2_params.yaml')
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
