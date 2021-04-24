#!/bin/bash


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

    # 実行コマンド作成
    _cmd="rosbag record \
            /zed_node/left/image_rect_color \
            /zed_node/right/image_rect_color \
            /zed_node/left/camera_info \
            /zed_node/rgb/image_rect_color \
            /zed_node/depth/depth_registered \
            /zed_node/rgb/camera_info \
            /tf \
            /tf_static \
            /clock \
            -O ${SAVE_FILEPATH}/${NOW_DATE}_rtabmap_zed"
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
            -O ${SAVE_FILEPATH}/${NOW_DATE}_rtabmap_rs"
    
    echo "実行コマンド:
$_cmd"
    
    # 実行 
    source ~/.bashrc
    $_cmd 
fi
