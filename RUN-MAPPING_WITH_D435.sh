#!/bin/bash

<< COMMENTOUT
Overview:
----------
RealSense Cameraを使ったRTAB-MAPによるSLAMを実行するときに実行するファイル.



Usage:
----------
1. Realsense CameraとROSのwrapperを起動
    $ bash RUN-MAPPING_WITH_D435.sh launch_rs
2.zed_wrapper_rosでのパラメータを再設定
    $ bash RUN-MAPPING_WITH_ZED.sh change_params
3. (option) RTAB-MAP用のrosbagを起動
    $ bash RUN-MAPPING_WITH_ZED.sh rosbag
4. RTAB-MAPを起動
    $ bash RUN-MAPPING_WITH_ZED.sh slam



Autor:
----------
onigirimal

COMMENTOUT





## ZED Cameraを立ち上げる
if [ "${1}" = "launch_rs" ]; then

# readonly CMD_LAUNCH_D435="roslaunch realsense2_camera rs_rgbd.launch align_depth:=true"
# D435i
readonly CMD_LAUNCH_D435="roslaunch realsense2_camera rs_camera.launch 
depth_width:=1280
depth_height:=720
depth_fps:=15
infra_width:=1280
infra_height:=720
color_width:=1280
color_height:=720
color_fps:=15
align_depth:=true
unite_imu_method:="linear_interpolation" 
enable_gyro:=true
enable_accel:=true"

PROMPT="
----------------------------------------
(1) RealSense Cameraを起動します.
$ ${CMD_LAUNCH_D435}

start   : s
quit    : q
----------------------------------------
>> "

while true; 
  do
    
    # ユーザーからの入力を待つ
    read -n 1 -s -p "${PROMPT}" input;
    # 入力した文字を表示
    echo "${input}"
    
    # quit
    if [ ${input} = "q" ]; then
      echo "quit"
      break;
    
    # プログラムを実行
    elif [ ${input} = "s" ]; then
      echo "start"
      source ~/realsense_ws/devel/setup.bash
      echo "${CMD_LAUNCH_D435}"  
      sleep 1
      ${CMD_LAUNCH_D435}
    fi  
 
  done






## ZED Cameraのパラメータを再設定
elif [ "${1}" = "launch_rs2" ]; then

readonly CMD_LAUNCH_D435="roslaunch realsense2_camera rs_rgbd2.launch align_depth:=true"

PROMPT="
----------------------------------------
(1) RealSense Cameraを起動します.
$ ${CMD_LAUNCH_D435}

start   : s
quit    : q
----------------------------------------
>> "

while true; 
  do
    
    # ユーザーからの入力を待つ
    read -n 1 -s -p "${PROMPT}" input;
    # 入力した文字を表示
    echo "${input}"
    
    # quit
    if [ ${input} = "q" ]; then
      echo "quit"
      break;
    
    # プログラムを実行
    elif [ ${input} = "s" ]; then
      echo "start"
      source ~/realsense_ws/devel/setup.bash
      echo "${CMD_LAUNCH_D435}"  
      sleep 1
      ${CMD_LAUNCH_D435}
    fi  
 
  done




## rosbag
elif [ "${1}" = "rosbag" ]; then
readonly CMD_ROSBAG="bash RUN-ROSBAG_RECORD.sh rs"
# echo "${CMD_ROSBAG}"
PROMPT="
----------------------------------------
(3) rosbagを. 
$ ${CMD_ROSBAG}

start   : s
quit    : q
----------------------------------------
>> "

while true; 
  do
    
    # ユーザーからの入力を待つ
    read -n 1 -s -p "${PROMPT}" input;
    # 入力した文字を表示
    echo "${input}"
    
    # quit
    if [ ${input} = "q" ]; then
      echo "break"
      break;
    
    # プログラムを実行
    elif [ ${input} = "s" ]; then
      echo "start"
      ${CMD_ROSBAG}
      sleep 1
    fi  
 
  done







## RTAB-MAPを起動
elif [ "${1}" = "slam" ]; then

#

<< COMMENTOUT
readonly CMD_SLAM="roslaunch rtabmap_ros mapping_zed.launch \
rtabmap_args:="--delete_db_on_start" \
database_path:=${PWD}/data.db \
rgb_topic:=/zed_node/rgb/image_rect_color \
depth_topic:=/zed_node/depth/depth_registered \
camera_info_topic:=/zed_node/rgb/camera_info \
frame_id:=base_link \
approx_sync:=false \
use_sim_time:=true \
rviz:=true"
COMMENTOUT

readonly CMD_SLAM="roslaunch rtabmap_ros mapping_d435.launch \
rtabmap_args:="--delete_db_on_start" \
database_path:=${PWD}/realsense.db \
depth_topic:=/camera/aligned_depth_to_color/image_raw \
rgb_topic:=/camera/color/image_raw \
camera_info_topic:=/camera/color/camera_info \
approx_sync:=false \
use_sim_time:=true \
rviz:=false \
rtabmapviz:=true
"


PROMPT="
----------------------------------------
(4) RTAB-MAPを開始します. 
$ ${CMD_SLAM}

start   : s
quit    : q
----------------------------------------
>> "

while true; 
  do
    
    # ユーザーからの入力を待つ
    read -n 1 -s -p "${PROMPT}" input;
    # 入力した文字を表示
    echo "${input}"
    
    # quit
    if [ ${input} = "q" ]; then
      echo "break"
      break;
    
    # プログラムを実行
    elif [ ${input} = "s" ]; then
      echo "start"
      source ../../devel/setup.bash
      ${CMD_SLAM}
      sleep 1
    fi  
 
  done


fi
