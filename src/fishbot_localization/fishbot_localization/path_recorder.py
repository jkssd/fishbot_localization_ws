#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
path_recorder.py
实时记录机器人走过的轨迹并发布为 nav_msgs/Path。

原理：
  - 订阅 TF 中的 map -> base_footprint 变换；
  - 其中 map -> odom 由 AMCL（Navigation2 定位）提供，
    odom -> base_footprint 由 robot_localization(EKF 融合 /odom + /imu) 提供；
  - 因此这条轨迹 = EKF 里程计 + AMCL 全局定位 的融合结果（多算法）。
  - 发布到 /robot_trajectory，在 RViz2 中用 Path 显示查看。
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')

        # use_sim_time 需在节点初始化早期声明，时钟才会正确使用 /clock
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('min_distance', 0.01)  # 相邻点最小距离(m)，用于去重

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        publish_rate = self.get_parameter('publish_rate').value
        self.min_distance = self.get_parameter('min_distance').value

        # TF 监听
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(Path, '/robot_trajectory', 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.path = Path()
        self.path.header.frame_id = self.map_frame

        self.get_logger().info(
            f'PathRecorder started: {self.map_frame} -> {self.base_frame}, '
            f'publishing on /robot_trajectory'
        )

    def timer_callback(self):
        try:
            # rclpy.time.Time() (时间戳为 0) 表示获取最新的可用变换
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            # TF 尚未就绪，跳过这一帧
            return

        now = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = trans.transform.translation.x
        pose.pose.position.y = trans.transform.translation.y
        pose.pose.position.z = trans.transform.translation.z
        pose.pose.orientation = trans.transform.rotation

        # 去重：原地不动时不重复堆积轨迹点
        if self.path.poses:
            last = self.path.poses[-1].pose.position
            dist = math.hypot(pose.pose.position.x - last.x,
                              pose.pose.position.y - last.y)
            if dist < self.min_distance:
                return

        self.path.header.stamp = now
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 使用 try_shutdown 而非 shutdown：
        # ros2 launch 收到 SIGINT 时 rclpy 已 shutdown 过一次 context，
        # 再次 shutdown 会抛 RCLError("rcl_shutdown already called")。
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
