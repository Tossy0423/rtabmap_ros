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

_cmd="roslaunch rtabmap_ros mapping_zed.launch \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/zed_node/rgb/image_rect_color \
    depth_topic:=/zed_node/depth/depth_registered \
    camera_info_topic:=/zed_node/rgb/camera_info \
    frame_id:=base_link \
    approx_sync:=false \
    use_sim_time:=true \
    rviz:=true
"
# 再生する時
# rosbag play -r 1.0 -l --clock <bagfile_path>


echo "実行コマンド: 
${_cmd}"

sleep 1

# 実行
$_cmd

