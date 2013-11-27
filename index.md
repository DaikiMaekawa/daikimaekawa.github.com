---
layout: page
title: Home
tagline: 
---
{% include JB/setup %}

## ブログ主の自己紹介

ソフト開発歴10年未満のひよっこプログラマー

好奇心旺盛で幅広く学んでいるため、非常に大まかですが

GUIアプリケーション開発、システム設計、ロボット運動制御系の開発等を得意としています。

趣味でコンピュータビジョン、3Dゲーム、組み込み系ソフトウェア、人工知能(行動計画/機械学習等)の開発を手がけています。

いくつかの開発プロジェクトに参加していますが、その中でも最も力を注いでいるのが[RoboCup](http://www.youtube.com/watch?v=Ua2hu3cCNFw "RoboCup")ですね。

2050年までに人型ロボットで「FIFA WorldCup」のチャンピオンチームに勝利するという目標を掲げ、その過程で人と技術を育てることを目的としている世界規模のランドマークプロジェクトです。(ロボコンと混同されがちですが別物)

↓我々CITBrainsは毎年行われる世界大会で優秀な成績を残し続けています。

[CITBrainsのメインページ](https://sites.google.com/site/hayashibaralab/robocup "CITBrainsのメインページ")

## このブログについて

私自身の備忘録としてや後輩への資料として、ソフトウェア開発に関する情報を不定期に更新します。

このブログはGitHub Pages + jekyllで運営されています。

このサイトのRSSを購読する -> [RSS Feed](http://daikimaekawa.github.io/rss.xml "RSS Feed")

## Posts List

<ul class="posts">
  {% for post in site.posts %}
    <li><span>{{ post.date | date_to_string }}</span> &raquo; <a href="{{ BASE_PATH }}{{ post.url }}">{{ post.title }}</a></li>
  {% endfor %}
</ul>

This theme is still unfinished. If you'd like to be added as a contributor, [please fork](http://github.com/plusjade/jekyll-bootstrap)!
We need to clean up the themes, make theme usage guides with theme-specific markup examples.

