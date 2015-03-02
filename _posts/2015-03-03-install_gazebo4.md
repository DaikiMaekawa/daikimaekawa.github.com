---
title: "特定のバージョンのGazeboを使用する方法"
layout: post
category : ROS
tagline: 
tags : [ROS, Gazebo]
---

{% include JB/setup %}

```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main" > /etc/apt/sources.list.d/gazebo-latest.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo aptitude install ros-indigo-gazebo4-ros-pkgs
```

