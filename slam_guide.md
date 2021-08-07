# 実機のロボットを使ってSLAMを実行
## 構成
- kobuki
- RealSenseD435i x1
- 2D-LiDAR x1

## 実行手順
1. kobukiとつなぐ
    ```shell
    roslaunch turtlebot_bringup minimal.launch
    ```

2. Rvizを開く
    ```shell
    rviz -d kobuki_debug.rviz
    ```

3. 2D-LiDARをつなぐ
    ```shell
    roslaunch urg_node urg_lidar.launch
    ```

4. RealSenseD435iをつなぐ 
    ```shell
    roslaunch realsense2_camera rs_camera.launch depth_width:=1280 depth_height:=720 depth_fps:=15 infra_width:=1280 infra_height:=720 color_width:=1280 color_height:=720 color_fps:=15 align_depth:=true tf_prefix:="front_rs" unite_imu_method:="linear_interpolation" enable_gyro:=true enable_accel:=true
    ```

5. MadgwickFilterを動かす
    ```shell
    rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
    ```

6. RealSenseD435iを`base_footprint`を親リンクとして座標変換
    ```shell
    rosrun tf2_ros static_transform_blisher 0 0 0.84 0 0 0 base_footprint front_rs_link
    ```

> **Note:**
    地図作成をオフラインで実行するときには事前にrosbagファイルを作成する必要がある. 地図作成に必要なtopicを実行コマンドは以下の通り. <br>
    ```
    rosbag record /tf /tf_static /clock /scan /odom /camera/aligned_depth_to_color/image_raw /camera/color/camera_info /camera/color/image_raw /rtabmap/imu  -O <filepath>
    ``` <br>
    作成したrosbagファイルからSLAMを実行する場合は以下のコマンドを実行. <br>
    ```
    rosbag play --clock --pause <filepath of rosbag>
    ```

7. SLAM
    ```shell
    roslaunch rtabmap_ros demo_turtlebot_mapping2.launch depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info database_path:=<filepath>
    ```

