# razgrom_bringup — оффлайн SLAM по записи робота-собаки (ROS 2 Jazzy)

## Установка (один раз, заранее)
```bash
sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-nav2-map-server
mkdir -p ~/razgrom_ws/src && cd ~/razgrom_ws/src
# этот пакет -> ~/razgrom_ws/src/razgrom_bringup
# FAST-LIO для ROS 2 (одно из):
git clone -b ROS2 --recursive https://github.com/hku-mars/FAST_LIO.git      # ветка ROS2 официального репозитория
# или: git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git
cd ~/razgrom_ws && source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
Отдельный workspace — чтобы не смешивать с дипломом. Каждый новый терминал: `source ~/razgrom_ws/install/setup.bash`.

## На соревновании
1. Разведка (5–15 мин):
   ```bash
   ros2 bag info <bag>
   ros2 run razgrom_bringup inspect_bag.py <bag>
   ```
   Выписать: топик облака, IMU, камеры, camera_info, base_frame, фрейм ноговой одометрии, есть ли время по точкам.
2. Быстрый старт (только RTAB-Map, лидар):
   ```bash
   ros2 launch razgrom_bringup slam.launch.py odom_source:=icp cloud_topic:=<облако> imu_topic:=<imu> base_frame:=<base>
   ros2 bag play <bag> --clock -r 0.5     # второй терминал; -r 0.5 – запас по скорости
   ```
   Если ICP теряется на качке, то `odom_source:=leg leg_odom_frame:=<odom>` (ICP с подсказкой от ног). Последний запасной вариант — `odom_source:=leg_only`.
3. Основной вариант (FAST-LIO → RTAB-Map):
   - скопировать `config/fast_lio_template.yaml` в `src/FAST_LIO/config/razgrom.yaml`, подставить топики, тип и число лучей лидара, extrinsic из /tf_static; пересобрать FAST-LIO;
   - терминал 1: `ros2 launch fast_lio mapping.launch.py config_file:=razgrom.yaml use_sim_time:=true rviz:=false`
   - терминал 2: `ros2 launch razgrom_bringup slam.launch.py odom_source:=fastlio body_to_base:="x y z yaw pitch roll" imu_topic:=<imu>`
   - терминал 3: `ros2 bag play <bag> --clock`
   body_to_base — положение base_link в фрейме IMU (обратное к tf base_link→imu из inspect_bag). Если IMU стоит в base_link без поворота, это просто минус смещение.
4. Камера: добавить `use_camera:=true rgb_topic:=... camera_info_topic:=...` (визуальные петли, Reg/Strategy=2). Нужен tf камеры.
5. Пивоты-маркеры: в `config/rtabmap_common.yaml` выставить `RGBD/MarkerDetection: "true"`, словарь и размер.
6. Локализация на готовой карте (второй прогон):
   `ros2 launch razgrom_bringup slam.launch.py localization:=true <те же аргументы>` + `ros2 bag play <bag> --clock`.
7. Экспорт: `ros2 run razgrom_bringup export_results.sh ~/.ros/rtabmap.db ~/razgrom_results`; граф и петли — `rtabmap-databaseViewer ~/.ros/rtabmap.db`.

## Что крутить, если плохо
| Симптом | Параметр (config/rtabmap_common.yaml) |
| мало петель | RGBD/ProximityBySpace true, Icp/CorrespondenceRatio ↓ 0.1, RGBD/OptimizeMaxError ↑ |
| ложные петли, карта «сминается» | Icp/CorrespondenceRatio ↑ 0.3, RGBD/OptimizeMaxError ↓ 1.0 |
| ICP-одометрия теряется | Icp/VoxelSize ↓ 0.05, Odom/ScanKeyFrameThr ↑ 0.6, odom_source:=leg |
| медленно | Icp/VoxelSize ↑ 0.2, RGBD/LinearUpdate ↑ 0.3, bag play -r 0.3 |
| пол/потолок в 2D-карте | Grid/MaxGroundHeight, Grid/MaxObstacleHeight (высота от base_link!) |
| «призраки» людей | Grid/RayTracing true; 3D – отдельный скрипт YOLO/ERASOR |
| ошибки TF / «could not get transform» | base_frame, /tf_static, body_to_base; use_sim_time везде; play с --clock |

Имена всех параметров и значения по умолчанию: `ros2 run rtabmap_slam rtabmap --params`.
