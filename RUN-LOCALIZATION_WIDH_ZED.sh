#!/bin/bash

# Load setup.bash
source ~/rtabmap_ws/devel/setup.bash

_cmd="roslaunch rtabmap_ros localization_zed.launch \
depth_topic:=/zed_node/depth/depth_registered \
rgb_topic:=/zed_node/rgb/image_rect_color \
camera_info_topic:=/zed_node/rgb/camera_info \
frame_id:=base_link \
approx_sync:=true \
use_sim_time:=true \
rviz:=true \
rtabmapviz:=false \
localization:=true \
database_path:=${PWD}/realsense.db \
"
# 再生する時
# rosbag play -r 1.0 -l --clock <bagfile_path>


echo "実行コマンド
${_cmd}"

sleep 1

# 実行
$_cmd

