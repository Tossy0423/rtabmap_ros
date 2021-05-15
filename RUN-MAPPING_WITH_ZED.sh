#!/bin/bash


# child0
# $ roslaunch zed_wrapper zed_no_tf.launch
# child1
# $ rosrun dynamic_reconfigure dynparam set zed_node depth_confidence 99
# $ rosrun dynamic_reconfigure dynparam set zed_node depth_texture_conf 90
# $ rosrun dynamic_reconfigure dynparam set zed_node depth_confidence 100
# child2
# $ bash RUN-ROSBAG-RECORD.sh zed
# child3
# RTAB-MAPSLAM

echo "${1}"



## ZED Cameraを立ち上げる
if [ "${1}" = "launch_zed" ]; then

readonly CMD_LAUNCH_ZED="roslaunch zed_wrapper zed_no_tf.launch"
echo "${CMD_LAUNCH_ZED}"

PROMPT="
----------------------------------------
ZED Cameraを起動します.
$ ${CMD_LAUNCH_ZED}

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
      echo "${CMD_LAUNCH_ZED}"  
      sleep 1
      ${CMD_LAUNCH_ZED}
    fi  
 
  done






## ZED Cameraのパラメータを再設定
elif [ "${1}" = "change_params" ]; then

readonly CHANGE_PARAM1="rosrun dynamic_reconfigure dynparam set zed_node depth_confidence 99"
readonly CHANGE_PARAM2="rosrun dynamic_reconfigure dynparam set zed_node depth_texture_conf 90"
readonly CHANGE_PARAM3="rosrun dynamic_reconfigure dynparam set zed_node depth_confidence 100"
# echo "${CHANGE_PARAM1}
# ${CHANGE_PARAM2}
# ${CHANGE_PARAM3}"


# echo "${CHANGE_PARAM1}
# ${CHANGE_PARAM2}
# ${CHANGE_PARAM3}"

PROMPT="
----------------------------------------
ZED Camera Parameterを再設定します.
$ ${CHANGE_PARAM1}
$ ${CHANGE_PARAM2}
$ ${CHANGE_PARAM3}

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
      echo "${CHANGE_PARAM1}"  
      ${CHANGE_PARAM1}
      sleep 3
      echo "${CHANGE_PARAM2}"  
      ${CHANGE_PARAM2}
      sleep 3
      echo "${CHANGE_PARAM2}"  
      ${CHANGE_PARAM2}
    #   sleep 1
    fi  
 
  done




## rosbag
elif [ "${1}" = "rosbag" ]; then
readonly CMD_ROSBAG="bash RUN-ROSBAG-RECORD.sh zed"
echo "${CMD_ROSBAG}"







## RTAB-MAPを起動
elif [ "${1}" = "slam" ]; then

#
readonly CMD_SLAM="roslaunch rtabmap_ros mapping_zed.launch \
rtabmap_args:="--delete_db_on_start" \
database_path:=${PWD}/data.db \
rgb_topic:=/zed_node/rgb/image_rect_color \
depth_topic:=/zed_node/depth/depth_registered \
camera_info_topic:=/zed_node/rgb/camera_info \
frame_id:=base_link \
approx_sync:=false \
use_sim_time:=true \
rviz:=false"


PROMPT="
----------------------------------------
RTAB-MAPを開始します. 
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
      ${CMD_SLAM}
      sleep 1
    fi  
 
  done


fi