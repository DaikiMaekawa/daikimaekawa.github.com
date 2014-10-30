---
layout: page
title: Home
tagline:
---
{% include JB/setup %}

##ブログ主の自己紹介

ロボット開発への熱い情熱と世界最高峰の技術が集結したチーム「CITBrains」のリーダー。

ロボカップにおける自律サッカーヒューマノイドの開発をしています。

 * 2014年度世界大会でルイヴィトンベストヒューマノイドを獲得

 * 2013年度テクニカルチャレンジ世界一位

 * 2012年度Kidサイズ総合世界一位

 * 日本大会5連覇中

<script src="//platform.linkedin.com/in.js" type="text/javascript"></script>
<script type="IN/MemberProfile" data-id="http://www.linkedin.com/in/daikimaekawa" data-format="inline" data-related="false"></script>

**********

## このブログについて

主にロボット関係ですがソフトウェア開発に関する情報を不定期に更新します。

このブログはGitHub Pages + jekyllで運営されています。

このサイトのRSSを購読する -> [RSS Feed](http://daikimaekawa.github.io/rss.xml "RSS Feed")

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

This theme is still unfinished. If you'd like to be added as a contributor, [please fork](http://github.com/plusjade/jekyll-bootstrap)!
We need to clean up the themes, make theme usage guides with theme-specific markup examples.

