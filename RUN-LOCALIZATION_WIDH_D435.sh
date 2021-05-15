#!/bin/bash

# Load setup.bash
source ~/rtabmap_ws/devel/setup.bash

_cmd="roslaunch rtabmap_ros localization_d435.launch \
depth_topic:=/camera/aligned_depth_to_color/image_raw \
rgb_topic:=/camera/color/image_raw \
camera_info_topic:=/camera/color/camera_info \
approx_sync:=false \
use_sim_time:=false \
rviz:=true \
rtabmapviz:=false \
localization:=true \
database_path:=/home/robo20/rtabmap_ws/src/rtabmap_ros/3f3.db \
"
# 再生する時
# rosbag play -r 1.0 -l --clock <bagfile_path>


echo "実行コマンド
${_cmd}"

sleep 1

# 実行
$_cmd

