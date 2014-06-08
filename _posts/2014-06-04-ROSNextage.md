---
title: "MoveIt! + RVizでグラフィカルなモーションプランニング環境"
layout: post
category : ROS
tagline: <p>NEXTAGEで遊ぼう01</p>
tags : [ROS, MoveIt!]
---

{% include JB/setup %} 

##開発環境

| OS                    | 　　　 | ROS ver |
|:--------------------- | ------ |:------- |
| Ubuntu12.04LTS(64bit) | 　　　 | Hydro   |

**********

## はじめに

6月14日に開催する[第二回ROS勉強会](http://ros-users.doorkeeper.jp/events/11230)はROS対応版NEXTAGE2台を使ってハッカソンを行う予定です。

そこで、NEXTAGEハッカソンに参加される方向けに、MoveIt!を使ってNEXTAGEを制御する方法について解説していきたいと思います。

NEXTAGEを動かすための環境構築やシステム全体の構成に関しては、第二回ROS勉強会の解説役を引き受けてくださった近藤さんの記事が
非常にわかりやすいです。

以下のURLをご参照ください。

http://www.youtalk.jp/post/85823518143/setup-nextage-ros

http://www.youtalk.jp/post/86079413783/ros-topic-service

http://www.youtalk.jp/post/86493954928/make-ros-package

## MoveIt!のユーザーインタフェース

![overview]({{ BASE_PATH }}/images/moveit/overview.jpg)

[rtmros_nextage](https://github.com/tork-a/rtmros_nextage)パッケージではUserInterfaceを実装するだけで動作させられる状態になっている
ので、そこを理解すればNEXTAGEハッカソンでとりあえず動かせるようになります。
全体の詳しい解説は後ほど行うとして、まずはUserInterfaceの解説をします。

ユーザーは３つの方法のいずれかでmove_groupが提供するアクションやサービスにアクセスすることができます。

 * move_group_interfaceパッケージを用いてC++で書く

 * moveit_commanderパッケージを用いてPythonで書く

 * Motion Planningプラグインを用いてRVizでグラフィカルに制御

今回はMoveIt!のRVizプラグインを使って、GUI上で目標値を設定してモーションプランニングを実行する方法について解説します。

## NEXTAGEを動かしてみよう

まずはシミュレータを起動しましょう。

    $ rtmlaunch nextage_ros_bridge nextage_ros_bridge_simulation.launch

![hrpsys_viewer]( {{BASE_PATH}}/images/moveit/hrpsys_viewer.jpg)

次にシミュレータ(または実機)のNEXTAGEにRViz上から指示を出すために以下のlaunchファイルを起動します。

    $ roslaunch nextage_moveit_config moveit_planning_execution.launch

![rviz_view]( {{BASE_PATH}}/images/moveit/rviz_view.jpg)

NEXTAGEのモデルが表示され、目標値をグラフィカルに入力できる状態でRVizが立ち上がりました。

球体や矢印をマウスで動かして目標値を変更するとプランニング結果がリアルタイムで表示されることが確認できます。(黄色のアーム)

![set_goal]( {{BASE_PATH}}/images/moveit/rviz_nextage_set_goal_l.jpg)

プランを実行するためにはExecuteボタンを押す必要があります。
![move_l]( {{BASE_PATH}}/images/moveit/rviz_nextage_move_l.jpg)

プランを実行することでMoveGroupActionGoalメッセージが発行されてシミュレータ(または実機)のアームが動作します。

![hrpsys_and_rviz]({{BASE_PATH}}/images/moveit/hrpsys_rviz_nextage_move_l.jpg)

Planning Groupをright_armに変更することで右腕も操作できるようになります。

![change_planning_group]({{BASE_PATH}}/images/moveit/rviz_nextage_change_planning_group.jpg)

![move_r]( {{BASE_PATH}}/images/moveit/hrpsys_rviz_nextage_move_r.jpg)

衝突が発生すると二つのリンクが赤く表示されます。(この時、プランは実行できない)

![collision]( {{BASE_PATH}}/images/moveit/rviz_nextage_collision.jpg)

Planned PathのLoop Animationを有効にすることで
開始状態から終了状態までのアームの軌跡を繰り返し表示させることができます。

また、Show Trailを有効にすると経路計画を以下の画像のように可視化することもできます。

![trail]({{BASE_PATH}}/images/moveit/rviz_nextage_trail.jpg)

以上でRVizからNEXTAGEのアームを制御するための解説を終わります。

次回はプログラム中でアームの目標値を設定する方法について解説する予定です。

