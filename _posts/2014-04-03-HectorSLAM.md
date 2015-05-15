---
title: "ROSでオドメトリフリーなSLAMを動かしてみよう"
layout: post
category : ROS
tagline: 
tags : [ROS, SLAM]
---

{% include JB/setup %} 

##開発環境

### OS

 * Ubuntu12.04

### ROS ver

 * Hydro

### 深度センサ

 * Xtion Pro Live

********** 

OpenNIはライセンス的に今度どうなっていくのか非常に不安ですし、ついにXtionが供給終了になりましたね…

なんか久しぶりにXtionで遊びたくなってきたのでhector_slamを使って地図でも描かせようと思います。

hector_slamはURG等の高レートが出せるLRFを生かしてオドメトリフリーなSLAMを実現します。

更にロール軸とピッチ軸のずれに対しても頑健に作られており、ロバストな動作が期待できる点で優れています。

Xtionはスペック的に心配ですがどこまでできるのか確かめてみましょう。

まず必要なものをインストール

    sudo aptitude install ros-hydro-openni2-launch
    sudo aptitude install ros-hydro-hector-slam-launch
    sudo aptitude install ros-hydro-depthimage-to-laserscan

view_framesでopenni2起動時のtfデータを可視化します。

    rosrun tf view_frames

camera_linkがベースになっているようですのでそこにbase_frameを親フレームとして追加し、mapまでのフレームを関連付ける必要があります。

static_transform_publisherを使えば簡単ですね。 ROS便利すぎ!

変換後の図は以下のとおり

![frames]({{ BASE_PATH }}/images/hector_slam/frames.png)

hector_slamを動かすにはLaserScanが必要なのでdepthimage_to_laserscanを用いて変換します。

下準備はこれだけ、後は使うだけです。

個別に起動するのは面倒なのでlaunchファイル作りました。

[launchファイルをダウンロード](https://github.com/DaikiMaekawa/hector_slam_example)

    roslaunch hector_openni.launch

一応動作時のシステム図も載っけときます。

    rqt_graph

![rosgraph]({{ BASE_PATH }}/images/hector_slam/rosgraph_openni.png)

XtionとPCを用意すればこれだけで地図が描けます。

<iframe width="640" height="360" src="//www.youtube.com/embed/tGUzG2srefI" frameborder="0" allowfullscreen></iframe>

オドメトリ無しで綺麗に地図が描けるのはすばらしいのですがやはり広い空間で試してみるとうまくいかない…

作成者もKinectとXtionでは精度が出なかったためURGをお勧めすると言ってますから今度はそちらで試してみましょう。

