---
layout: page
title: Home
tagline: 
---
{% include JB/setup %}

<style>
#followButton {
    color: #121516;
    text-shadow: 0 1px 1px #ccc;
    padding: 0.2em 0.4em;
    
    -moz-border-radius: 5px;
    -webkit-border-radius: 5px;
    border-radius: 5px;
    -moz-background-clip: padding; -webkit-background-clip: padding-box; background-clip: padding-box;

    -moz-box-shadow: 0px 0px 2px #666;
    -webkit-box-shadow: 0px 0px 2px #666;
    box-shadow: 0px 0px 2px #666;

    background-color: #eeeeee;
    background-image: -webkit-gradient(linear, left top, left bottom, from(#eeeeee), to(#aaaaaa));
    background-image: -webkit-linear-gradient(top, #eeeeee, #aaaaaa);
    background-image: -moz-linear-gradient(top, #eeeeee, #aaaaaa);
    background-image: -ms-linear-gradient(top, #eeeeee, #aaaaaa);
    background-image: -o-linear-gradient(top, #eeeeee, #aaaaaa);
    background-image: linear-gradient(top, #eeeeee, #aaaaaa);
    filter: progid:DXImageTransform.Microsoft.gradient(startColorStr='#eeeeee', EndColorStr='#aaaaaa');
}
#followButton img { position: relative; top: 2px; margin: -6px 6px 0 0; }
</style>  

##ブログ主の自己紹介

ロボット開発への熱い情熱と世界最高峰の技術が集結したチーム「CITBrains」のリーダー。

ロボカップ世界一を目指して自律ヒューマノイドロボットを開発しています。

ロボカップとは2050年までに完全自律の人型ロボットで「FIFA WorldCup」のチャンピオンチームに勝利するという目標を掲げ、その過程で人と技術を育てることを目的としている世界規模のランドマークプロジェクトです。(ロボコンと混同されがちですが別物)

我々CITBrainsは毎年行われる世界大会で優秀な成績を残し続けています -> [CITBrainsのメインページ](https://sites.google.com/site/hayashibaralab/robocup "CITBrainsのメインページ")

ちなみに私の担当は主に人工知能、GUIアプリケーションの開発です。

この他には趣味でWebアプリケーション、コンピュータビジョン、3Dゲーム、組み込み系ソフトウェア等の開発を手がけています。

![gravator](https://2.gravatar.com/avatar/41da189ffc08a2ed46e9a1416f4db136?d=https%3A%2F%2Fidenticons.github.com%2F726114dd8781c27fa4551d9af7886827.png&r=x&s=100)

<a href="https://github.com/DaikiMaekawa" 
    title="Follow DaikiMaekawa on Github" 
    id="followButton" class="noBg"> 
    <img src="https://github.com/favicon.ico" width="18" height="18" />
    Follow me on Github
</a>
　　[![endorse](https://api.coderwall.com/daikimaekawa/endorsecount.png)](https://coderwall.com/daikimaekawa)

<a href="http://qiita.com/DaikiMaekawa" 
    title="Follow DaikiMaekawa on Qiita" 
    id="followButton" class="noBg"> 
    <img src="http://qiita.com/favicon.ico" width="18" height="18" />
    Follow me on Qiita
</a> 

**********

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

