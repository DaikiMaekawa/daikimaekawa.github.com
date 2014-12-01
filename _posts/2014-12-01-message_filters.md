---
title: "message_filtersでタイムスタンプがおおよそ一致した際にコールバックさせる方法"
layout: post
category : ROS
tagline:
tags : [ROS, message_filters]
---

{% include JB/setup %}

### はじめに

message_filtersでタイムスタンプがおおよそ一致した際にコールバックを呼び出したい場合にはApproximateTimeポリシーを使用することで実現できます。

### message_filters

まず、message_filterをご存知無い方のために簡単に説明します。

message_filtersはROSのメッセージに対して手軽にフィルタリングを適用するためのライブラリでroscppとrospyから使用することができます。

例えば、複数のソースからの異なる型のメッセージのタイムスタンプを監視し、一致したときに呼び出させるコールバック関数を設定することが可能です。

詳しくは以下のURLを参照して下さい。 

http://wiki.ros.org/message_filters

### ApproximateTime Policy

サンプルコードを載せておきます。

```cpp
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

void syncMsgsCB(const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::ImuConstPtr &imu){
    
}

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> SyncPolicy;

int main(int argc, char *argv[]){
    ros::init(argc, argv, "approximate_time_tutorial");
    ros::NodeHandle nh;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 1);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "imu", 1);
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), odom_sub, imu_sub);
    sync.registerCallback(boost::bind(&syncMsgsCB, _1, _2));

    ros::spin();
    
    return 0;
}

```

アルゴリズムの詳細は[こちら](http://wiki.ros.org/message_filters/ApproximateTime)

