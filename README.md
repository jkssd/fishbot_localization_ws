# fishbot 定位工程（AMCL 地图定位 + EKF 里程计 + 轨迹记录）

基于鱼香 ROS fishbot 仿真的 ROS2 Humble 定位工程，实现：

- **地图定位**：AMCL（Navigation2）在 `room` 占据栅格地图上定位机器人，发布 `map -> odom`。
- **EKF 里程计**：`robot_localization` 融合 `/odom`（差速轮）+ `/imu`（IMU），发布 `odom -> base_footprint`。
- **轨迹记录**：`path_recorder` 监听 `map -> base_footprint`，发布 `/robot_trajectory`（`nav_msgs/Path`）。
- **可视化**：RViz2 同时显示地图、AMCL 粒子云、机器人、轨迹、EKF 里程计。

## 目录结构

```
chapt7_ws_localization/
├── src/
│   ├── fishbot_description/    # 机器人 URDF / Gazebo 世界 / 控制器（差速、IMU、激光、GPS）
│   └── fishbot_localization/   # 定位整合包（本工程核心）
│       ├── launch/navigation_localization.launch.py  # 整合 launch
│       ├── config/ekf.yaml                  # 局部 EKF 配置
│       ├── config/rviz/navigation_ekf.rviz  # RViz 配置
        ├── maps #地图文件与地图配置文件
│       └── fishbot_localization/path_recorder.py  # 轨迹记录节点
```

## 依赖

ROS2 Humble + `nav2` + `robot_localization` + `slam_toolbox` + `gazebo_ros` 等。

## 编译与运行

```bash
cd chapt7_ws_localization
colcon build
source install/setup.bash

# 终端 1：启动 Gazebo 仿真（机器人 + 传感器 + 差速控制器）
ros2 launch fishbot_description gazebo_sim.launch.py

# 终端 2：地图定位(AMCL) + EKF + 轨迹 + RViz2
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch fishbot_localization navigation_localization.launch.py

# 终端 3：键盘控制机器人移动，观察地图上的轨迹
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 关键约定

- IMU 话题：`/imu`；里程计：`/odom`；激光：`/scan`；地图：`/map`；轨迹：`/robot_trajectory`。
- 坐标系：`map -> odom -> base_footprint -> base_link`。
- 差速控制器已关闭 `enable_odom_tf`，`odom -> base_footprint` 由 EKF 发布，避免 TF 冲突。
