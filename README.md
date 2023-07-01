# auto-rccar-toy

## はじめに

本ソフトはRCカーを自律走行させるソフトである  
ローカライゼーションはRTK-GNSSのみで、ナビゲーションはNav2を使用  
方向検出や障害物検出にCameraのIMUやDepthも使用している  

<table>
<tr>
<td><img width="240" src="./assets/image/DSC_0967.JPG"></td>
<td><img width="240" src="./assets/image/DSC_0968.JPG"></td>
<td><img width="240" src="./assets/image/DSC_0969.JPG"></td>
</tr>
<tr>
<td><img width="240" src="./assets/image/DSC_0962.JPG"></td>
<td><img width="240" src="./assets/image/DSC_0965.JPG"></td>
<td><img width="240" src="./assets/image/DSC_0966.JPG"></td>
</tr>
</table>

## 動作環境

- ハードウェア

  - 1/10RC メルセデス・ベンツ ウニモグ 406 (CC-02シャーシ)
  - OP.1895 タミヤ ブラシレスモーター 02 センサー付 21.5T
  - タミヤ ブラシレス エレクトロニック スピードコントローラー 04SR センサー付
  - Jetson Xavier NX
  - M5 ATOM Matrix
  - Ublox F9P
  - Ublox アンテナ
  - Realsense D435i
  - バッテリー
  - バッテリー

  - その他配線や部品

- ソフトウェア

  - ROS2 Foxy
  - micro-ROS
  - RTKLIB

  - m5stack-rccar-toy

  - ROS2 Package
    - micro_ros_agent
    - joy_linux
    - teleop_twist_key
    - teleop_twist_joy

## 構成図

T.B.A

## セットアップ手順

1. リポジトリをクローンする

    ```bash
    git clone https://github.com/endusers/auto-rccar-toy.git
    ```

1. セットアップスクリプトを実行する

    ```bash
    ./auto-rccar-toy/scripts/setup.sh -w ワークスペースディレクトリ -r
    ```

    ※ 作成するワークスペースのパスを -w で指定してください  
    ※ RTKLIBのインストールが不要の場合は -r の指定を削除してください  
    ※ RTKLIBのインストール先は ~/RTKLIB 固定です  

1. ビルドする

    ```bash
    cd ワークスペースディレクトリ
    colcon build --symlink-install
    ```

1. RTKLIBの設定をする(RTKLIBをインストールした場合)

    本設定は実機環境でRTKLIBを使って動かす場合に必要です  
    シミュレーション環境で動かす場合は本設定は不要です  

    1. 任意のテキストエディタで下記スクリプトを開く

        ```bash
        nano ワークスペースディレクトリ/rtklib-str2str.sh
        ```

    1. ntripの必要箇所を設定する

        ```sh
        ./str2str -in ntrip://[user[:passwd]@]addr[:port][/mntpnt] -out serial://ttyACM0:460800 -b 1
        ```

## シミュレーション環境

1. シミュレーション環境を立ち上げる

    ```bash
    ros2 launch rccar_bringup rccar-simulation.launch.xml world:=smalltown.world
    ```

    ※初回起動はエラーになるため、Gazeboが立ち上がった後に、再度立ち上げなおしてください  

1. ローカライゼーションを立ち上げる

    ```bash
    ros2 launch rccar_robot_localization rccar-localization.launch.xml use_sim_time:=true
    ```

1. ナビゲーションを立ち上げる

    ```bash
    ros2 launch rccar_navigation2 rccar_navigation2.launch.py use_sim_time:=true map:=smalltown_world.yaml
    ```

1. Rviz2を立ち上げる

    ```bash
    ros2 launch rccar_bringup rccar-rviz.launch.xml
    ```

※ワークスペースディレクトリへのパスの指定は省略しています  
※ターミナル起動毎に設定、または、 .bashrc に追加して対応してください  

## 実機環境

1. RCカーの電源をONする

1. JetsonにSSHで接続しRTKLIBを立ち上げる

    ```bash
    cd ワークスペースディレクトリ
    ./rtklib-str2str.sh
    ```

1. JetsonにSSHで接続しロボットを立ち上げる

    ```bash
    ros2 launch rccar_bringup rccar-robot.launch.xml
    ```

1. JetsonにSSHで接続しカメラを立ち上げる

    ```bash
    ros2 launch rccar_bringup rccar-camera.launch.xml
    ```

1. JetsonにSSHで接続しローカライゼーションを立ち上げる

    ```bash
    ros2 launch rccar_robot_localization rccar-localization.launch.xml
    ```

1. JetsonにSSHで接続しナビゲーションを立ち上げる

    ```bash
    ros2 launch rccar_navigation2 rccar_navigation2.launch.py
    ```

1. ホストPCでRviz2を立ち上げる

    ```bash
    ros2 launch rccar_bringup rccar-rviz.launch.xml
    ```

※ワークスペースディレクトリへのパスの指定は省略しています  
※ターミナル起動毎に設定、または、 .bashrc に追加して対応してください  
