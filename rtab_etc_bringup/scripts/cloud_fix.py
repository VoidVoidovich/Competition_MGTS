#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Ретранслятор PointCloud2 с исправлением заголовка (row_step/width), опционально – публикация tf из nav_msgs/Odometry.

    ros2 run <пакет> cloud_fix.py --ros-args -p use_sim_time:=true \
        -p in_topic:=/points_raw -p out_topic:=/points_fixed [-p frame_id:=velodyne] [-p odom_topic:=/state_SDK]

Зачем: в некоторых записях (Leg-KILO) у облака row_step = 0, и RTAB-Map падает на проверке
data.size() == row_step * height. Узел пересчитывает row_step = len(data) / height (и width при необходимости).
Если задан odom_topic – публикует tf odom.header.frame_id -> odom.child_frame_id (для режима odom_source:=leg).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class CloudFix(Node):
    def __init__(self):
        super().__init__('cloud_fix')
        self.declare_parameter('in_topic', '/points_raw')
        self.declare_parameter('out_topic', '/points_fixed')
        self.declare_parameter('frame_id', '')          # пусто = не менять
        self.declare_parameter('odom_topic', '')        # пусто = tf не публиковать
        self.declare_parameter('odom_child_frame', '')  # пусто = из сообщения
        g = lambda n: self.get_parameter(n).get_parameter_value().string_value
        self.frame = g('frame_id'); self.child = g('odom_child_frame')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PointCloud2, g('out_topic'), qos)
        self.create_subscription(PointCloud2, g('in_topic'), self.cb, qos)
        self.warned = False
        if g('odom_topic'):
            self.tfb = TransformBroadcaster(self)
            self.create_subscription(Odometry, g('odom_topic'), self.odom_cb, 50)
        self.get_logger().info('cloud_fix: %s -> %s' % (g('in_topic'), g('out_topic')))

    def cb(self, m: PointCloud2):
        h = max(1, m.height)
        if m.point_step > 0 and (m.width * m.point_step * h != len(m.data)):
            m.width = len(m.data) // (m.point_step * h)
        m.height = h
        m.row_step = m.width * m.point_step
        if len(m.data) != m.row_step * m.height:
            m.data = bytes(m.data[:m.row_step * m.height])   # отрезать «хвост», если размер не кратен точке
        if self.frame: m.header.frame_id = self.frame
        if not self.warned:
            self.get_logger().info('исправлено: width=%d point_step=%d row_step=%d, поля=%s, frame=%s' %
                                   (m.width, m.point_step, m.row_step, [f.name for f in m.fields], m.header.frame_id))
            self.warned = True
        self.pub.publish(m)

    def odom_cb(self, o: Odometry):
        t = TransformStamped()
        t.header = o.header
        t.child_frame_id = self.child or o.child_frame_id
        t.transform.translation.x = o.pose.pose.position.x
        t.transform.translation.y = o.pose.pose.position.y
        t.transform.translation.z = o.pose.pose.position.z
        t.transform.rotation = o.pose.pose.orientation
        self.tfb.sendTransform(t)


def main():
    rclpy.init(); n = CloudFix()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
