# -*- coding: utf-8 -*-
"""
GNSS/RTK 定位（双 EKF + navsat）+ 静态地图参照 + Nav2 无地图导航 + 轨迹显示

架构：
  - ekf_filter_node_odom（robot_localization）: 融合 /odom + /imu，发布 odom -> base_footprint
  - ekf_filter_node_map（robot_localization）: 融合 /odom(速度) + /imu(航向) + /odometry/gps，发布 map -> odom
  - navsat_rtk_driver: 把原始 /gps/fix 转成带 RTK 状态与厘米协方差的 /gps/rtk_fix
  - navsat_transform（robot_localization）: 把 /gps/rtk_fix 经纬度转成 map 系 /odometry/gps
  - map_server: 加载 maps/room.yaml 作为静态地图参照（map_static 帧，仅可视化，不参与导航）
  - static_transform_publisher: 用 map_yaw 把 map_static 对齐到 EKF 的 map 帧
  - navigation_launch.py（nav2）: 无地图导航栈（规划/控制/代价地图），不含 AMCL / map_server
  - path_recorder: 监听 map -> base_footprint，发布 /robot_trajectory 轨迹
  - rviz2: 显示静态地图、机器人精确位姿、激光、GNSS 定位轨迹

运行方式（先启动仿真，再启动本文件）：
  ros2 launch fishbot_description gazebo_sim.launch.py
  ros2 launch fishbot_localization navigation_localization.launch.py
  ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 控制机器人，观察 GNSS 定位轨迹

标定：
  若轨迹与静态地图有整体旋转错位，用 map_yaw 微调（弧度）：
  ros2 launch fishbot_localization navigation_localization.launch.py map_yaw:=1.57
"""
import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # ---------- 路径 ----------
    fishbot_localization_path = get_package_share_directory('fishbot_localization')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_param_path = os.path.join(fishbot_localization_path, 'config', 'nav2_params.yaml')
    ekf_param_path = os.path.join(fishbot_localization_path, 'config', 'dual_ekf_navsat.yaml')
    map_yaml_path = os.path.join(fishbot_localization_path, 'maps', 'room.yaml')
    rviz2_config_dir = os.path.join(
        fishbot_localization_path, 'config', 'rviz', 'navigation_ekf.rviz')

    # ---------- 启动参数 ----------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value=use_sim_time,
        description='Use simulation (Gazebo) clock if true')

    map_yaw = LaunchConfiguration('map_yaw', default='0.0')
    declare_map_yaw = DeclareLaunchArgument(
        'map_yaw', default_value=map_yaw,
        description='静态地图(map_static)相对 EKF map 帧的 yaw 标定值(rad)')

    # ---------- 0. 静态地图参照（纯可视化，用于对照定位轨迹）----------
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'yaml_filename': map_yaml_path,
                     'frame_id': 'map_static'}],
    )

    lifecycle_manager_map_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': ['map_server']}],
    )

    static_tf_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_map_static',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', map_yaw, '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map', '--child-frame-id', 'map_static'],
    )

    # ---------- 1. 局部 EKF：发布 odom -> base_footprint 与 /odometry/local ----------
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_param_path],
        remappings=[('odometry/filtered', 'odometry/local')],
    )

    # ---------- 2. 全局 EKF：发布 map -> odom 与 /odometry/global ----------
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_param_path],
        remappings=[('odometry/filtered', 'odometry/global')],
    )

    # ---------- 3. GNSS/RTK 驱动：/gps/fix -> /gps/rtk_fix（写入 RTK 状态与厘米协方差）----------
    navsat_rtk_driver_node = Node(
        package='fishbot_localization',
        executable='navsat_rtk_driver',
        name='navsat_rtk_driver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---------- 4. navsat_transform：/gps/rtk_fix -> /odometry/gps（map 系绝对位置）----------
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_param_path],
        remappings=[('imu/data', '/imu'),
                    ('gps/fix', '/gps/rtk_fix'),
                    ('gps/filtered', '/gps/filtered'),
                    ('odometry/gps', '/odometry/gps'),
                    ('odometry/filtered', '/odometry/global')],
    )

    # ---------- 5. Nav2 无地图导航栈（规划/控制/代价地图 + lifecycle_manager）----------
    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, '/launch', '/navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path,
            'autostart': 'true',
        }.items(),
    )

    # ---------- 6. 轨迹记录：发布 /robot_trajectory ----------
    path_recorder_node = Node(
        package='fishbot_localization',
        executable='path_recorder',
        name='path_recorder',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---------- 7. RViz2 ----------
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
        declare_map_yaw,
        # RViz2 在部分 GPU 驱动下 GLSL 着色器编译失败（"active samplers ... same texture
        # image unit"），导致 Map/RobotModel 等显示插件无法渲染甚至崩溃。强制软件渲染规避。
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        map_server_node,
        lifecycle_manager_map_node,
        static_tf_map_node,
        ekf_local_node,
        ekf_global_node,
        navsat_rtk_driver_node,
        navsat_node,
        launch_navigation,
        path_recorder_node,
        rviz2_node,
    ])
