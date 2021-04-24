#!/bin/bash

# Load setup.bash
source ~/rtabmap_ws/devel/setup.bash

# 
# readonly SETTING_FILE_PATH="Examples/RGB-D/D435.yaml"
# echo "Setting file path            >>  ${SETTING_FILE_PATH}"

# readonly SUBSCRIBE_IMAGE_COLOR="/camera/rgb/image_raw:=/camera/color/image_raw"
# echo "Subscribe image color topic  >>  ${SUBSCRIBE_IMAGE_COLOR}"

# readonly SUBSCRIBE_IMAGE_DEPTH="/camera/depth_registered/image_raw:=/camera/aligned_depth_to_color/image_raw"
# echo "Subscribe image depth topic  >>  ${SUBSCRIBE_IMAGE_DEPTH}"

# 実行コマンド作成
# _cmd="rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt ${SETTING_FILE_PATH} ${SUBSCRIBE_IMAGE_COLOR} ${SUBSCRIBE_IMAGE_DEPTH}"

_cmd="roslaunch rtabmap_ros mapping_d435.launch \
rtabmap_args:="--delete_db_on_start" \
depth_topic:=/camera/aligned_depth_to_color/image_raw \
rgb_topic:=/camera/color/image_raw \
camera_info_topic:=/camera/color/camera_info \
approx_sync:=false \
use_sim_time:=true \
rviz:=true \
rtabmapviz:=true
"
# 再生する時
# rosbag play -r 1.0 -l --clock <bagfile_path>


echo "実行コマンド
${_cmd}"

sleep 1

# 実行
$_cmd

