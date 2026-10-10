# fishbot 定位工程（多传感器 EKF + GNSS/RTK 全局定位 + 无地图室外导航）

基于鱼香 ROS fishbot 仿真的 ROS2 Humble 定位工程，用 **多传感器 EKF + GNSS/RTK** 取代原来的 AMCL 地图定位，实现户外（无地图）环境下的全局定位与轨迹记录，并保留 Nav2 无地图室外导航能力。

## 核心改动（相对原 AMCL 版本）

- **移除 AMCL / map_saver**：定位改用 GNSS/RTK + EKF，不做粒子滤波。但**重新引入 `map_server` 作为静态地图参照**：加载 `maps/room.yaml`（`map_static` 帧），仅用于 RViz 可视化对照，不参与导航代价地图。
- **双 EKF + navsat（robot_localization）**：
  - `ekf_filter_node_odom`：局部 EKF，融合 `/odom`（轮式里程计）+ `/imu`，发布 `odom -> base_footprint` 与 `/odometry/local`。
  - `ekf_filter_node_map`：全局 EKF，融合 `/odom`(速度) + `/imu`(航向) + `/odometry/gps`(绝对位置)，发布 `map -> odom` 与 `/odometry/global`。
  - `navsat_transform`：把 `/gps/rtk_fix` 的经纬度（WGS84）转成 map 系坐标，发布 `/odometry/gps` 与 `/gps/filtered`。
- **GNSS/RTK 传感器与驱动**：`fishbot_description` 已加入 Gazebo GPS 传感器，以低噪声（水平 ~1cm、垂直 ~2cm）输出原始 `/gps/fix`；`navsat_rtk_driver` 节点把原始 GNSS 转成带 RTK 状态与厘米级协方差的 `/gps/rtk_fix`，模拟 **RTK 固定解**。
- **轨迹记录**：`path_recorder` 监听 `map -> base_footprint`，发布 `/robot_trajectory`。
- **崩溃修复**：Gazebo 世界 `spherical_coordinates` 原为 `latitude_deg=0 / longitude_deg=0`（赤道/本初子午线），落在 UTM 分带边界，导致 `navsat_transform` 抛 `GeographicLib::GeographicErr` 崩溃。已改为真实地点（上海 `31.2304 / 121.4737`）。

## 坐标系

```
map  <- 全局 EKF（融合 GNSS 绝对位置，离散跳变）
  └── odom  <- 局部 EKF（融合轮式里程计 + IMU，连续无跳变）
        └── base_footprint  <- 差速控制器 / URDF
              └── base_link
                    └── imu_link / laser_link / camera_link ...
```

- 差速控制器已关闭 `enable_odom_tf`，`odom -> base_footprint` 由局部 EKF 发布，避免 TF 冲突。

## 话题约定

| 话题 | 类型 | 说明 |
|------|------|------|
| `/odom` | `nav_msgs/Odometry` | 差速轮式里程计（odom 系） |
| `/imu` | `sensor_msgs/Imu` | IMU 数据 |
| `/scan` | `sensor_msgs/LaserScan` | 激光雷达 |
| `/gps/fix` | `sensor_msgs/NavSatFix` | GNSS 原始经纬度 |
| `/gps/rtk_fix` | `sensor_msgs/NavSatFix` | RTK 固定解（带厘米协方差，navsat 输入） |
| `/odometry/gps` | `nav_msgs/Odometry` | GPS 转换后的 map 系绝对位置 |
| `/odometry/global` | `nav_msgs/Odometry` | 全局 EKF 输出（map 系） |
| `/odometry/local` | `nav_msgs/Odometry` | 局部 EKF 输出（odom 系） |
| `/gps/filtered` | `sensor_msgs/NavSatFix` | 滤波后经纬度（可视化） |
| `/robot_trajectory` | `nav_msgs/Path` | 机器人走过的轨迹 |

## 目录结构

```
chapt7_ws_localization/
├── src/
│   ├── fishbot_description/    # 机器人 URDF / Gazebo 世界 / 控制器（差速、IMU、激光、GPS/RTK）
│   └── fishbot_localization/   # 定位整合包（本工程核心）
│       ├── launch/navigation_localization.launch.py  # 整合 launch
│       ├── config/dual_ekf_navsat.yaml               # 双 EKF + navsat 配置
│       ├── config/nav2_params.yaml                   # Nav2 无地图导航配置
│       ├── config/rviz/navigation_ekf.rviz           # RViz 配置
│       ├── maps/                                     # 地图文件（保留，供可选 SLAM/调试使用）
│       └── fishbot_localization/path_recorder.py     # 轨迹记录节点
```

## 依赖

ROS2 Humble + `nav2` + `robot_localization` + `gazebo_ros` + `ros-humble-gazebo-plugins`（提供 Gazebo GPS 传感器插件）等。

## 编译与运行

```bash
cd chapt7_ws_localization
colcon build
source install/setup.bash

# 终端 1：启动 Gazebo 仿真（机器人 + 差速控制器 + IMU + 激光 + GPS/RTK）
ros2 launch fishbot_description gazebo_sim.launch.py

# 终端 2：GNSS/RTK 定位（双 EKF + navsat）+ Nav2 无地图导航 + 轨迹 + RViz2
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch fishbot_localization navigation_localization.launch.py

# 终端 3：键盘控制机器人移动，观察 map 系 GNSS 定位轨迹
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> 提示：`navsat_transform` 以**第一帧 GPS 作为原点**（`wait_for_datum: false`），因此机器人起始位置即 map 原点。等待约 3 秒（`delay: 3.0`）后全局 EKF 收敛，RViz 中 `map` 系轨迹即与机器人真实位置对齐。

> 静态地图标定：若轨迹与静态地图有整体旋转错位，用 `map_yaw`（弧度）微调——`ros2 launch fishbot_localization navigation_localization.launch.py map_yaw:=1.57`，直到激光点云贴合墙壁。

## 验证方法

1. RViz2 中 `Fixed Frame` 为 `map`，观察：
   - `Map`（静态地图参照）是否正常显示；
   - 机器人模型是否精确贴合地图自由区域移动（定位结果来自 GNSS/EKF 的 `map -> odom -> base` 链，而非里程计轨迹）；
   - `GNSS 定位轨迹`（绿色，`/robot_trajectory`）是否随键盘控制而更新。
2. 命令行查看定位输出：
   ```bash
   ros2 topic echo /odometry/gps        # GPS 绝对位置（map 系）
   ros2 topic echo /odometry/global     # 全局 EKF 融合结果
   ros2 topic echo /gps/filtered        # 滤波后经纬度
   ```

## 从仿真迁移到真实硬件（RTK）

仿真中 Gazebo 直接输出 `/gps/fix`。真实硬件只需将 GNSS/RTK 接收机的驱动输出重映射到同一话题即可，其余配置不变：

- 常见方案：`nmea_navsat_driver` 或 u-blox（`ublox_gps`）驱动，输出 `sensor_msgs/NavSatFix` 到 `/gps/fix`。
- RTK 固定解时 `NavSatFix.status.status = STATUS_GBAS_FIX / STATUS_RTK_FIX`，若未启用 `use_fixed_covariance`，`navsat_transform` 会使用消息自带协方差。
- 若 RTK 接收机输出 ENU 速度，可额外在全局 EKF 中融合（参考 `odom1` 配置扩展 vx/vy）。
