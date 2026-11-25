# -*- coding: utf-8 -*-
"""RTAB-Map для оффлайн-обработки записи робота-собаки.

Режимы одометрии (odom_source):
  fastlio  – одометрия от FAST-LIO (/Odometry, облако /cloud_registered_body, фрейм body). Основной вариант.
  icp      – лидарная одометрия RTAB-Map (icp_odometry) по сырому облаку.
  leg      – icp_odometry с начальным приближением из ноговой одометрии робота (tf leg_odom_frame -> base_frame).
  leg_only – только ноговая одометрия из tf (запасной вариант, если ICP не сходится).
localization:=true – режим локализации по готовой базе (карта не дополняется).

Пример:
  ros2 launch rtab_etc_bringup slam.launch.py odom_source:=icp cloud_topic:=/points imu_topic:=/imu base_frame:=base_link
  ros2 bag play <bag> --clock        (в другом терминале, после запуска)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

ARGS = {
    'odom_source': ('icp', 'fastlio | icp | leg | leg_only'),
    'localization': ('false', 'true = локализация по готовой базе'),
    'database_path': ('~/.ros/rtabmap.db', 'файл базы RTAB-Map'),
    'base_frame': ('base_link', 'фрейм корпуса робота из /tf_static'),
    'leg_odom_frame': ('odom', 'фрейм ноговой одометрии робота (tf odom -> base_link)'),
    'cloud_topic': ('/lidar/points', 'PointCloud2 3D-лидара'),
    'imu_topic': ('/imu/data', 'sensor_msgs/Imu'),
    'use_imu': ('true', 'подписываться на IMU'),
    'use_camera': ('false', 'подключить камеру (визуальные петли)'),
    'rgb_topic': ('/camera/image_raw', 'изображение'),
    'camera_info_topic': ('/camera/camera_info', 'параметры камеры'),
    'fastlio_odom_topic': ('/Odometry', 'одометрия FAST-LIO'),
    'fastlio_cloud_topic': ('/cloud_registered_body', 'облако FAST-LIO в фрейме body'),
    # положение base_frame в фрейме IMU (body FAST-LIO) – взять из /tf_static (base_link -> imu, инвертировать)
    'body_to_base': ('0 0 0 0 0 0', 'x y z yaw pitch roll: body(IMU) -> base_frame'),
    'gui': ('true', 'запустить rtabmap_viz'),
}


def setup(context):
    g = lambda k: LC(k).perform(context)
    share = get_package_share_directory('rtab_etc_bringup')
    common = os.path.join(share, 'config', 'rtabmap_common.yaml')
    icp_yaml = os.path.join(share, 'config', 'icp_odometry.yaml')
    src = g('odom_source'); loc = g('localization').lower() == 'true'
    cam = g('use_camera').lower() == 'true'; imu = g('use_imu').lower() == 'true'
    base = g('base_frame'); nodes = [LogInfo(msg='razgrom: odom_source=%s localization=%s camera=%s' % (src, loc, cam))]

    # ---- параметры узла rtabmap
    p = {'database_path': os.path.expanduser(g('database_path')), 'subscribe_scan_cloud': True,
         'subscribe_depth': False, 'subscribe_rgb': cam, 'subscribe_odom_info': src in ('icp', 'leg'),
         'wait_imu_to_init': imu, 'publish_tf': True}
    remap = [('rgb/image', g('rgb_topic')), ('rgb/camera_info', g('camera_info_topic'))]
    if imu: remap.append(('imu', g('imu_topic')))
    if cam: p['Reg/Strategy'] = '2'          # визуальная + ICP

    if src == 'fastlio':
        p.update(frame_id='body')
        remap += [('scan_cloud', g('fastlio_cloud_topic')), ('odom', g('fastlio_odom_topic'))]
        xyzypr = g('body_to_base').split()
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher', name='body_to_base',
                          arguments=['--x', xyzypr[0], '--y', xyzypr[1], '--z', xyzypr[2], '--yaw', xyzypr[3],
                                     '--pitch', xyzypr[4], '--roll', xyzypr[5], '--frame-id', 'body', '--child-frame-id', base],
                          parameters=[{'use_sim_time': True}]))
    elif src in ('icp', 'leg'):
        p.update(frame_id=base)
        oparams = {'frame_id': base, 'odom_frame_id': 'icp_odom', 'publish_tf': True, 'wait_imu_to_init': imu}
        if src == 'leg': oparams['guess_frame_id'] = g('leg_odom_frame')
        oremap = [('scan_cloud', g('cloud_topic'))] + ([('imu', g('imu_topic'))] if imu else [])
        nodes.append(Node(package='rtabmap_odom', executable='icp_odometry', name='icp_odometry', output='screen',
                          parameters=[common, icp_yaml, oparams], remappings=oremap))
        remap += [('scan_cloud', g('cloud_topic'))]
    elif src == 'leg_only':
        p.update(frame_id=base, odom_frame_id=g('leg_odom_frame'))
        remap += [('scan_cloud', g('cloud_topic'))]
    else:
        raise RuntimeError('odom_source должен быть fastlio | icp | leg | leg_only')

    if loc:
        p.update({'Mem/IncrementalMemory': 'false', 'Mem/InitWMWithAllNodes': 'true'})
        args = []
    else:
        args = ['-d']                         # начать новую базу (старая удаляется!)
    nodes.append(Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
                      parameters=[common, p], remappings=remap, arguments=args))
    if g('gui').lower() == 'true':
        nodes.append(Node(package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz', output='screen',
                          parameters=[common, p], remappings=remap))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()] +
                             [OpaqueFunction(function=setup)])
