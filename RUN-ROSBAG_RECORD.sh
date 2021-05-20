#!/bin/bash

<< COMMENTOUT
Overview:
----------
RTAB-Map, ORB-SLAM2で使用することができる記録ファイル(rosbag)を作成するためのスクリプト.
このスクリプトで生成されたrosbagファイルを使うことで, 任意のタイミング, 各種パラメータをチューニングしてからもう一度SLAMを実行することができるので, 現地へ行ってカメラを回す必要がなくなる. 
このスクリプトで対応しているカメラは, RealSense D435, ZED Cameraの2種類.

< !!!!! 注意 Caution !!!!! >
RTAB-Map, ORB-SLAM2で扱うTopicは画像であるため, 長時間使用する際はファイルのサイズに注意.
各カメラにおける単位時間あたりのファイルサイズは以下の通り.
- RealSense D435    : 40MB/sec (2.4GB/min)
- ZED Camera        : 200MB/sec(12GB/min)



Usage:
----------
- RealSense D435
$ roslaunch realsense2_camera rs_camera.launch align_depth:=true
$ bash RUN-ROSBAG-RECORD.sh rs

- ZED camera
$ roslaunch zed_wrapper zed_no_tf.launch
$ rosrun dynamic_reconfigure dynparam set zed_node depth_confidence 99
$ rosrun dynamic_reconfigure dynparam set zed_node depth_texture_conf 90
$ rosrun dynamic_reconfigure dynparam set zed_node depth_confidence 100
$ bash RUN-ROSBAG-RECORD.sh zed


Autor:
----------
onigirimal

COMMENTOUT


function checkdir()
{   

    # 任意のディレクトリが存在するか確認
    # echo "args : $1"
    
    # 実行コマンドを作成
    _cmd="${1}"

    # dirがないときに処理するところ
    if [ ! -d $1 ]; then
        echo "bagdataを保存するディレクトリが見つかりませんでした."
        echo "保存ディレクトリを作成します. ($_cmd)"
        mkdir $_cmd
    fi
}


# ============================================================================ # 


# echo "実行時の引数の数 >> ${#}"

# 保存するディレクトリパス
SAVE_DIRPATH="/mnt/ssd1000"
# SAVE_DIRPATH=$PWD

# 保存するディレクトリ名
SAVE_DIRNAME="bagdata"

# bagファイルを保存するファイルパス
SAVE_FILEPATH="${SAVE_DIRPATH}/${SAVE_DIRNAME}"
echo "$SAVE_FILEPATH"

# bagdataデータ名とする実行時刻を取得
NOW_DATE=`date '+%Y.%m.%d-%H%M%S'`



# 引数が指定されていない or "-h"が渡された時
if [ $# = 0 ] || [ $1 = "-h" ]; then
    
    echo "引数を指定してください."
    echo "ZED Camera    : $ bash RUN-ROSBAG_RECORD.sh zed"
    echo "RealSenseD435 : $ bash RUN-ROSBAG_RECORD.sh rs"



# 引数がzedの場合
elif [ $1 = "zed" ]; then
    
    echo "ZED Camera"

    # 保存ディレクトリがあるかを確認
    checkdir $SAVE_FILEPATH

    echo "保存するディレクトリパス: 
$SAVE_DIRPATH"

    echo "保存するファイル名: 
${NOW_DATE}_zed.bag"
    

    echo "< !!!!! 注意 Caution !!!!! >"
    echo "ZED Cameraは, bagファイルが大きくなります."
    
    # どのTopicを記録するか選択
    read -n1 -p "
    0: ORB-SLAM2 only 
    1: RTAB-Map only
    2: ORB-SLAM2 & RTAB-Map
    " select_topic

    if [[ $select_topic = [0] ]]; then
        echo ""
        echo "ORB-SLAM2のみで使用できるTopicを取得します."
        echo "単位時間あたりのファイルサイズは, 100MB/sec(6GB/min)です."
        _cmd="rosbag record \
            /zed_node/left/image_rect_color \
            /zed_node/right/image_rect_color \
            /zed_node/left/camera_info \
            /tf \
            /tf_static \
            /clock \
            -O ${SAVE_FILEPATH}/${NOW_DATE}_ORBonly_zed"
    
    elif [[ $select_topic = [1] ]]; then
        echo ""
        echo "RTAB-Mapのみで使用できるTopicを取得します."
        echo "単位時間あたりのファイルサイズは, 100MB/sec(6GB/min)です."
        _cmd="rosbag record \
            /zed_node/depth/depth_registered \
            /zed_node/rgb/camera_info \
            /zed_node/rgb/image_rect_color \
            /tf \
            /tf_static \
            /clock \
            -O ${SAVE_FILEPATH}/${NOW_DATE}_RTABonly_zed"
    
    elif [[ $select_topic = [2] ]]; then
        echo ""
        echo "ORB-SLAM2とRTAB-Mapで使用できるTopicを取得します."
        echo "単位時間あたりのファイルサイズは, 200MB/sec(6GB/min)です."
        _cmd="rosbag record \
            /zed_node/left/image_rect_color \
            /zed_node/right/image_rect_color \
            /zed_node/left/camera_info \
            /zed_node/depth/depth_registered \
            /zed_node/rgb/camera_info \
            /zed_node/rgb/image_rect_color \
            /tf \
            /tf_static \
            /clock \
            -O ${SAVE_FILEPATH}/${NOW_DATE}_ORBandRTAB_zed"
    
    fi


    # 実行コマンド作成
    
    echo "実行コマンド: \
    $_cmd"
    

    sleep 1

    # 実行 
    source ~/.bashrc
    $_cmd



# 引数がrsの場合
elif [ $1 = "rs" ]; then

    echo "rs"
    
    # 保存ディレクトリがあるかを確認
    checkdir $SAVE_FILEPATH
    
    echo "保存するディレクトリパス: 
$SAVE_DIRPATH"

    echo "保存するファイル名: 
${NOW_DATE}_rs.bag"

    # 実行コマンド作成
    _cmd="rosbag record \
            /camera/aligned_depth_to_color/image_raw \
            /camera/color/image_raw \
            /camera/color/camera_info \
            /tf \
            /tf_static \
            -O ${SAVE_FILEPATH}/${NOW_DATE}_rs"
    
    echo "実行コマンド:
$_cmd"
    
    # 実行 
    source ~/.bashrc
    $_cmd 
fi
