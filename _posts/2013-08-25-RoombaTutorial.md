---
title: "RoombaのROSドライバをソースからビルドして使ってみる"
layout: post
category : ROS
tagline: "ROS入門講座01"
tags : [ROS, Roomba]
---

{% include JB/setup %}

動作環境： Ubuntu12.04, ROS Groovy

ROSを使ってRoombaを動かすために必要そうな情報を収集しているとroomba_500_seriesという便利そうなパッケージを発見しました。

しかし、サポートされているバージョンがfuerteまでなのでgroovyはソースからビルドするしかなさそうです。

    mkdir ~/ros_ws
    cd ~/ros_ws
    git clone https://github.com/Arkapravo/roomba_500_ROS_drivers
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_ws >> ~/.bashrc
    source ~/.bashrc

(注) すでにワークスペースがある場合はそこにソースを置いてください。

当然、catkinへの対応はまだなのでrosmakeでビルドしましょう。

    rosmake roomba_robot 
    rosmake serial_communication 

PCとRoombaをシリアルケーブルでつないだ後にポートを確認しましょう。

    dmesg | grep tty

すると以下の様に表示されます。

    [35675.368847] usb 1-1.2: FTDI USB Serial Device converter now attached to ttyUSB0

/dev/ttyUSB0へのアクセス権を取得します。

    sudo chmod 777 /dev/ttyUSB0

準備が整ったのでノードを起動します。

    rosrun roomba_500_series roomba500_light_node

このログが表示されれば成功です。

    [ INFO] [1377530011.590896574]: Roomba for ROS 2.01
    [ INFO] [1377530011.869933041]: Connected to Roomba.

geometry_msgs/Twistを発行してRoombaを旋回させます。

    rostopic pub -1 cmd_vel geometry_msgs/Twist '[0, 0, 0]' '[0, 0, 0.1]' && rostopic pub -1 cmd_vel geometry_msgs/Twist '[0, 0, 0]' '[0, 0, 0]'

