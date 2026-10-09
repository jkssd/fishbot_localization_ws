import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

#ros2 launch fishbot_navigation2 navigation2.launch.py

def generate_launch_description():
    fishbot_navigation_path=get_package_share_directory("fishbot_navigation2")
    nav2_bringup_dir=get_package_share_directory("nav2_bringup")
    rviz2_config_dir=os.path.join(nav2_bringup_dir,"rviz","nav2_default_view.rviz")

    use_sim_time=LaunchConfiguration("use_sim_time",default="true")
    map_yaml_path=LaunchConfiguration("map",default=os.path.join(fishbot_navigation_path,"maps","room.yaml"))
    nav2_param_path=LaunchConfiguration("params_file",default=os.path.join(fishbot_navigation_path,"config","nav2_params.yaml"))

    use_sim_time_description=DeclareLaunchArgument(
        name="use_sim_time",
        default_value=use_sim_time,
        description="Use simulation(Gazebo) clock if true"
    )

    map_yaml_path_description=DeclareLaunchArgument(
        name="map",
        default_value=map_yaml_path,
        description="Full path to map file to load"
    )

    nav2_param_path_description=DeclareLaunchArgument(
        name="params_file",
        default_value=nav2_param_path,
        description="Full path to param file to load"
    )

    launch_navigation2=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir,"/launch","/bringup_launch.py"],
        ),
        launch_arguments={
            "map":map_yaml_path,
            "use_sim_time":use_sim_time,
            "params_file":nav2_param_path
        }.items()
    )

    rviz2_node=Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d",rviz2_config_dir],
        parameters=[{"use_sim_time":use_sim_time}],
        output="screen"
    )

    return launch.LaunchDescription([
        use_sim_time_description,
        map_yaml_path_description,
        nav2_param_path_description,
        launch_navigation2,
        rviz2_node
    ])
