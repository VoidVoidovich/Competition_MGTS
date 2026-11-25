#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Первые 15 минут соревнования: что лежит в записи и какие аргументы подставить в slam.launch.py.

    ros2 run razgrom_bringup inspect_bag.py <папка_bag>

Печатает: топики, типы, число сообщений и частоту; дерево /tf_static с переводом в аргументы;
поля PointCloud2 (есть ли время по точкам – нужно FAST-LIO); фреймы IMU, камеры, одометрии.
"""
import sys, os, math, collections
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
meta = yaml.safe_load(open(os.path.join(bag, 'metadata.yaml')))['rosbag2_bagfile_information']
storage = meta.get('storage_identifier', 'mcap')
dur = meta['duration']['nanoseconds'] * 1e-9
print('Хранилище: %s, длительность %.1f с, сообщений %d' % (storage, dur, meta['message_count']))

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage), rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
counts = {t['topic_metadata']['name']: t['message_count'] for t in meta['topics_with_message_count']}
print('\n%-45s %-40s %8s %8s' % ('топик', 'тип', 'сообщ.', 'Гц'))
for name in sorted(types):
    n = counts.get(name, 0)
    print('%-45s %-40s %8d %8.1f' % (name, types[name], n, n / dur if dur else 0))

want = {'sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/Imu', 'sensor_msgs/msg/CameraInfo', 'nav_msgs/msg/Odometry',
        'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'}
first = {}; tf_static = []; tf_pairs = collections.Counter()
while reader.has_next():
    topic, data, t = reader.read_next()
    ty = types[topic]
    if ty == 'tf2_msgs/msg/TFMessage':
        msg = deserialize_message(data, get_message(ty))
        for tr in msg.transforms:
            if topic == '/tf_static': tf_static.append(tr)
            else: tf_pairs[(tr.header.frame_id, tr.child_frame_id)] += 1
    elif ty in want and topic not in first:
        first[topic] = deserialize_message(data, get_message(ty))
    if len(first) >= sum(1 for v in types.values() if v in want) and tf_static and len(tf_pairs) > 0:
        pass  # читаем до конца, чтобы собрать весь /tf (записи небольшие)

def ypr(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return yaw, pitch, roll

print('\n/tf_static (родитель -> потомок: x y z | yaw pitch roll, рад):')
if not tf_static: print('  НЕТ /tf_static – взаимное положение датчиков придётся задавать вручную!')
seen = set()
for tr in tf_static:
    k = (tr.header.frame_id, tr.child_frame_id)
    if k in seen: continue
    seen.add(k); t_ = tr.transform.translation
    print('  %s -> %s:  %.3f %.3f %.3f | %.3f %.3f %.3f' % (k[0], k[1], t_.x, t_.y, t_.z, *ypr(tr.transform.rotation)))
print('\n/tf (динамические пары): ' + (', '.join('%s->%s (%d)' % (a, b, n) for (a, b), n in tf_pairs.most_common()) or 'нет'))

print('\nДатчики:')
for topic, m in first.items():
    ty = types[topic]
    if ty.endswith('PointCloud2'):
        fields = [f.name for f in m.fields]
        tfield = [f for f in fields if f in ('t', 'time', 'timestamp', 'offset_time')]
        print('  ЛИДАР %s: frame=%s, %dx%d, поля=%s' % (topic, m.header.frame_id, m.height, m.width, fields))
        print('     время по точкам: %s' % (tfield or 'НЕТ – FAST-LIO будет хуже/не заработает, берите odom_source:=icp'))
        if 'ring' in fields: print('     есть ring – похоже на Velodyne/Ouster (lidar_type 2/3), число лучей = высота или max(ring)+1')
    elif ty.endswith('Imu'):
        print('  IMU %s: frame=%s' % (topic, m.header.frame_id))
    elif ty.endswith('CameraInfo'):
        print('  CAMERA_INFO %s: frame=%s, %dx%d, модель=%s' % (topic, m.header.frame_id, m.width, m.height, m.distortion_model))
    elif ty.endswith('Odometry'):
        print('  ODOM %s: %s -> %s (ноговая одометрия? -> leg_odom_frame)' % (topic, m.header.frame_id, m.child_frame_id))
    elif 'Image' in ty:
        print('  КАМЕРА %s: frame=%s (%s)' % (topic, m.header.frame_id, ty.split('/')[-1]))
print('\nДальше: подставьте топики и base_frame в slam.launch.py (см. README).')
