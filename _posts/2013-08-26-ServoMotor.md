---
title: "サーボモータの制御方法"
layout: post
category : SPPBoard
tagline: "dsPIC入門講座03"
tags : [dsPIC]
---

{% include JB/setup %}

##環境

    基盤       : SPPBoard(dsPIC30F3014使用)
    コンパイラ  : C30

##はじめに

今回はSPPBoardを用いてサーボモータ(S3003)を制御する方法を解説します。

S3003はPWM制御を使用することで簡単に制御することができます。(S3003はRCサーボ)

つまり、dsPIC入門講座02で解説したOutPutCompare(OC)モジュールを使えばよいのです。

しかし、SPPBoardに採用したdsPIC30F3014にはOC端子が二つしかありません。

車輪ロボット等を駆動させることを考えると最低二つのモータをそれぞれOC端子に割り当てることになりそうです。

その状態でS3003も同時に制御したい場合にはどうすればよいでしょうか?

ここでは、I/Oピンとタイマを組み合わせることによりPWM信号を生成する手法を紹介します。

##サーボモータの仕様書から必要な情報を読み取ろう

[仕様書を入手](http://www.es.co.th/schemetic/pdf/et-servo-s3003.pdf "S3003") 

###タイマの割り込み周期

S3003のパルス幅は0.5 ~ 2.3msです。

よって1.8msの間で0deg ~ 180deg動くことがわかります。

    (2.3 - 0.5) / 180 = 0.01ms で1deg動く

切りが良いので、0.01msをタイマの割り込み周期にします。

(注) S3003は180deg以上回転するので実際のパルス幅で測定すると0.55 ~ 2.35ms付近がちょうど0deg ~ 180degの範囲になっていました。

###角度制御用の信号

S3003が認識する周期幅が6 ~ 25msの範囲なので20msくらいの周期幅でHigh, Lowを切り替えます。

タイマの割り込み周期が0.01msなので2000カウントで20msとなります。

![FutabaServo]({{ BASE_PATH }}/images/FutabaServo.jpg)

**60degに制御する場合**

    0.01 x 60 = 0.6ms
    0.55msで0degなので
    0.6 + 0.55 = 1.15ms

タイマは0.01ms毎の割り込みなので

    High: 115カウント
    Low : 1885カウント

とする周期をサーボモータに送ることで60degに制御することができます。

**0degに制御する場合**

    High: 55カウント
    Low : 1945カウント

**180degに制御する場合**

    High: 255カウント
    Low : 1745カウント

タイマの割り込み周期を0.01msにしたのは1deg動かすために変化させるHighのカウント数を1にするためです。

##サーボモータを動かしてみよう

###角度指定用の関数

    void servo(int angle){
        angle += 145; //90 + 55
        ServoTargetValue = angle
    }

 - **angle += 145**  
   角度の座標系を0 ~ 180degから中心を0degとした-90 ~ 90degに変更

 - **ServoTargetValue**  
   タイマ1の方でHigh側のカウント上限に使用するグローバル変数

###初期化関数

    void ServoInitFunc(void){
        servo(0);
        ConfigIntTimer1(T1_INT_PRIOR_1 & T1_INT_ON);
        OpenTimer1(T1_ON & T1_GATE_OFF & T1_PS_1_1 & T1_SYNC_EXT_OFF & T1_SOURCE_INT, 200-1);
    }

 - **OpenTimer1(T1_ON & T1_GATE_OFF & T1_PS_1_1 & T1_SYNC_EXT_OFF & T1_SOURCE_INT, 200-1)**  
    4/80MHz x 1 x 200 = 0.01msec
    タイマの割り込み周期を0.01msecに設定

###タイマ関数

    void _ISR _T1Interrupt(void){
        IFS0bits.T1IF = 0;
        TimeCount++;

        if(TimeCount < ServoTargetValue){
            SERVO = 1;
        }else if(TimeCount <= 2000){
            TimeCount = 0;
        }
    }

サーボモータの信号端子にPWM信号を出力します。

タイマ資源が余っているとき限定ですがこの方法を使用することですべてIOピンにおいてPWM制御ができますので試してみてください。

以上でタイマを使ったサーボモータの制御についての解説を終わります。
