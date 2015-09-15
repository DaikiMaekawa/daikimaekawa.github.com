---
layout: page
title: Home
tagline:
---
{% include JB/setup %}

##ブログ主の自己紹介

大学を休学してRapyuta Roboticsで警備ドローンを開発しています。

日本にいた頃はros-jpの勉強会主催者をやっていました。

また、ロボカップの世界大会優勝チームCITBrainsやつくばチャレンジのProject ORNEを始め、複数のロボット開発プロジェクトに参加していました。

行雲流水の如く在るべき場所へと流れ行く

<script src="//platform.linkedin.com/in.js" type="text/javascript"></script>
<script type="IN/MemberProfile" data-id="http://www.linkedin.com/in/daikimaekawa" data-format="inline" data-related="false"></script>

**********

## このブログについて

主にロボット関係ですがソフトウェア開発に関する情報を不定期に更新します。

このブログはGitHub Pages + jekyllで運営されています。

[このサイトのRSSを購読する](http://daikimaekawa.github.io/rss.xml "RSS Feed")

###総訪問者数

<script language="Javascript">
document.write('<a href="http://www.f-counter.jp/k2/65/17/1386490417/"></a>');</script><noscript>
<a href=http://qhg.f-counter.com/>カウンター</a></noscript>
<a href=http://www.free-counter.jp/>
<img src="http://www.f-counter.net/j/17/1386490417/" alt="アクセスカウンター" border="0"></a>

## Posts List

<ul class="posts">
  {% for post in site.posts %}
    <li><span>{{ post.date | date_to_string }}</span> &raquo; <a href="{{ BASE_PATH }}{{ post.url }}">{{ post.title }}</a></li>
  {% endfor %}
</ul>

