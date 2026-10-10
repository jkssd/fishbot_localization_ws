#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
navsat_rtk_driver.py
GNSS/RTK 驱动节点：把原始 GNSS 定位（/gps/fix）转成带 RTK 高精度状态与
厘米级协方差的 NavSatFix（/gps/rtk_fix），供 navsat_transform_node 使用。

仿真中：Gazebo GPS 传感器输出原始 /gps/fix，本节点写入 RTK 固定解状态与
厘米级 position_covariance，模拟 RTK 接收机的输出。
真实硬件：可把 nmea/ublox 等驱动直接输出到 /gps/rtk_fix（或改订阅话题），
本节点即可被真实 RTK 接收机驱动替代。

坐标系：不依赖 TF，仅透传经纬度并改写状态/协方差。
"""
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus


class NavSatRtkDriver(Node):
    def __init__(self):
        super().__init__('navsat_rtk_driver')

        # RTK 精度（可调）：水平 ~1cm，垂直 ~2cm
        self.declare_parameter('horizontal_stddev', 0.01)
        self.declare_parameter('vertical_stddev', 0.02)
        h = self.get_parameter('horizontal_stddev').value
        v = self.get_parameter('vertical_stddev').value
        self.var_h = float(h) * float(h)
        self.var_v = float(v) * float(v)

        self.sub = self.create_subscription(NavSatFix, '/gps/fix', self.callback, 10)
        self.pub = self.create_publisher(NavSatFix, '/gps/rtk_fix', 10)

        self.get_logger().info(
            f'navsat_rtk_driver: /gps/fix -> /gps/rtk_fix '
            f'(RTK, cov=({self.var_h:.2e}, {self.var_v:.2e}))'
        )

    def callback(self, msg: NavSatFix):
        out = NavSatFix()
        out.header = msg.header
        out.header.frame_id = msg.header.frame_id or 'base_link'

        # Humble 的 sensor_msgs 无 STATUS_RTK_FIX，用 STATUS_GBAS_FIX（高精度差分修正）代替
        out.status.status = NavSatStatus.STATUS_GBAS_FIX
        out.status.service = NavSatStatus.SERVICE_GPS

        out.latitude = msg.latitude
        out.longitude = msg.longitude
        out.altitude = msg.altitude

        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        out.position_covariance = [
            self.var_h, 0.0, 0.0,
            0.0, self.var_h, 0.0,
            0.0, 0.0, self.var_v,
        ]

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = NavSatRtkDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
