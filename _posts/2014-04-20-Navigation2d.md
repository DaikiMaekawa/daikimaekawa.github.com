---
title: "hector_slamと2D Navigationを組み合わせてRoombaを自律移動させる"
layout: post
category : ROS
tagline: 
tags : [ROS, SLAM, Navigation]
---

{% include JB/setup %} 

##開発環境

### OS

 * Ubuntu12.04

### ROS ver

 * Hydro

### 距離センサ

 * UTM-30LX

### 移動台車

 * iRobot Create (500 series)

**********

## はじめに

今回はROSの中でも基礎中の基礎であるnavigationについて解説します。

navigationはセンサーの情報を元に目的地への経路を計画し台車に速度指令を出力します。

台車としては研究用の特別価格で[7万円で2台セットのRoomba](http://science.irobot-jp.com/)を購入して動かします。

## 概要

![overview]({{ BASE_PATH }}/images/navigation/overview.jpg)

move_baseはnavigation関係のプラグインを集め実体化しているものです。

それぞれのプラグインの定義は別々のパッケージに存在しますがどれもnav_coreのインタフェースを満たすように設計されています。

プラグインはパラメータになっているので名前を変更することで入れ替えが可能であり、この辺りを理解するにはpluginlibを学ぶことをおすすめします。

自己位置推定を行うamclは完全に独立しているのでこちらも自由に入れ替えることができます。

## ハードウェア要求

今回はRoombaを使うので関係ないですが任意のロボットにnavigationを適用する際には重要になってきます。

 * ディファレンシャル・ドライブかつホロノミック型のロボットであること

 * 車輪部のどこかに平面レーザがマウントされていること

 * ロボットの形が方形か円形であること

これら三つの要求は必須ではありませんが、パフォーマンスを向上させるために必要な要素です。

## 使い方

gmapping(オドメトリと距離センサの情報を使ったSLAMのパッケージ)で地図を作成し、amclで自己位置推定を行うのがROSの中ではもっとも一般的です。

しかし、Roombaのオドメトリは精度が悪くamclのパラメータ調整が非常にシビアなので今回はオドメトリを使用しません。

hector_slamを使用してオドメトリ無しで地図を描かせながら自律移動をさせてみましょう。

サンプルは[GitHubにアップ](https://github.com/DaikiMaekawa/ros-navigation2d-example)しました。

    $ roslaunch navigation2d_example move_base.launch

システム図

![rosgraph]({{ BASE_PATH }}/images/navigation/rosgraph.png)

オドメトリフリーと言いつつもlocal_plannerの計画のためにmove_baseにodomトピックを与えています。

まあ、自己位置推定はURGのみで行ってますけどね。

<iframe width="640" height="480" src="//www.youtube.com/embed/c68E9-21fkw" frameborder="0" allowfullscreen></iframe>

