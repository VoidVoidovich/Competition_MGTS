#!/usr/bin/env bash
# Экспорт результатов из базы RTAB-Map. Запуск: ros2 run razgrom_bringup export_results.sh [~/.ros/rtabmap.db] [out_dir]
# Точные ключи rtabmap-export проверьте заранее: rtabmap-export --help
set -e
DB=${1:-$HOME/.ros/rtabmap.db}
OUT=${2:-$PWD/razgrom_results}
mkdir -p "$OUT"; cd "$OUT"
echo "== 3D-облако (ply) и траектория"
rtabmap-export --cloud --voxel 0.05 --output_dir "$OUT" "$DB" || echo "!! облако: проверьте ключи rtabmap-export --help"
rtabmap-export --poses --output_dir "$OUT" "$DB" || echo "!! позы: проверьте ключи rtabmap-export --help"
echo "== 2D-карта: запустите slam.launch.py localization:=true, проиграйте пару секунд bag и выполните:"
echo "   ros2 run nav2_map_server map_saver_cli -f $OUT/map --ros-args -p use_sim_time:=true"
echo "== Просмотр графа/петель: rtabmap-databaseViewer $DB"
ls -la "$OUT"
