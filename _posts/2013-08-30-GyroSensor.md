---
title: "ジャイロセンサの制御方法"
layout: post
category : SPPBoard
tagline: "dsPIC入門講座04"
tags : [dsPIC, I2C]
---

{% include JB/setup %}

##開発環境

### 基盤

 * SPPBoard(dsPIC30F3014使用)  

### コンパイラ

 * C30  

### ジャイロセンサ

 * IMU3000  

**********

##はじめに

SPPBoard上でジャイロセンサ(IMU3000)を制御するプログラムを作成してみましょう。

IMU3000はI2Cインタフェースをサポートしているので、今回はI2C通信に関する基礎を解説したいと思います。

**********

##I2Cの基礎知識

###I2Cとは

I2CはInter Integrated Circuit(集積回路間通信)の略です。

I2Cはデータ線が送受信兼用で1本しかありません。

つまりI2Cの構成はデータ線+クロック線の2本となります。


###I2Cはシリアル通信

I2Cは同期式シリアル通信なので基準となるクロックパルスに合わせてデータ線へ信号を出力することになります。

###マスタとスレーブ

クロックパルスを送信する側のことを「マスタ」、送られてきたクロックパルスに従って動作する側のことを「スレーブ」といいます。

また、I2Cではクロックを送信するのは必ずマスタ側です。

###回路構成

I2Cのピンにはプルアップ抵抗が必要です。

これは様々なデバイスに対応するためにオープン・ドレイン端子となってためです。

I2Cのハードウェアは非常に単純です。

しかし、それを補うためプログラムは少し複雑になります。

###通信のタイミング

I2Cのデータバス(SDA)はプルアップされているので初期状態で1です。

クロック(SCL)も同様です。

    SCL: 1と0を交互に出力
    SDA: SCLが1の時状態を変化させてはいけない

(注) 通信の開始と終了の時だけは例外です

SCLが1の時

    SDA 1 -> 0 : スタート・コンディション(通信開始命令)
    SDA 0 -> 1 : ストップ・コンディション(通信停止命令)

これらの命令を発行できるのはマスタだけです。


データの送受信をする際、スレーブ側から応答が返ってきたことを確認しながらデータをやりとりする仕組みになっています。

    1. データの送受信の単位は8ビット
     -> 送信終了時にマスタはSDAを1に戻して受信待機状態

    2. スレーブが送信側になる
     -> 1ビットだけ0を送信(アクノリッジ信号)

マスタは8ビット送信後アクノリッジが来るまで待っていることで、常に正しく通信することができます。

例としてマスタが受信でスレーブが送信の場合を考えます。

    1. マスタが受信要求を出します
    2. スレーブからアクノリッジが返ってきます
    3. スレーが続けてデータを送信します
    4. マスタはこれを受信し、8ビット区切りでアクノリッジを返します

(注) スレーブ側では8ビット送信後、マスタがアクノリッジを返すまで待機します。

###アドレス送信の方式

I2Cでは一つのデータバスへ複数のデバイスを繋ぐことができます。

通信するスレーブはアドレスにより固定します。

    1. スタートコンディション発行
    2.「スレーブアドレス+要求」を送信

    要求はそのデバイスへ(から)の送信(受信)

アドレス指定には10ビットアドレスモードと7ビットアドレスモードの二つです。

 - **10ビットアドレスモード**  
 
   8ビットを2回に分けて送信する

   最初に送信する8ビットは上位5ビットは「1 1 1 1 0」
   残り3ビットはアドレスの上位2ビット+送受信指定

   次に送信する8ビットはすべてアドレスデータ

 - **7ビットアドレスモード**
 
   7ビットのアドレス+送受信指定

        送受信指定ビット
         | -> 送信なら0
         | -> 受信なら1

**********

##実際にプログラムにするにはどうするか

基本的にはC30コンパイラのI2C関連の関数を呼び出すだけでできます。

コンパイラのバグでそのままではライブラリがリンクされていませんので手動で行う必要があります。

その方法に関しては[このサイト](http://www2.ocn.ne.jp/~mhage/PIC_Trap/I2C.html "I2C")を参照してください。

勉強のためここではあえてライブラリを使用せずにレジスタを直接操作することによりI2C通信を実現しジャイロセンサを制御してみましょう。

**********

##仕様書からレジスタマップを読み解こう

[dsPIC30F3014の仕様書を入手](http://ww1.microchip.com/downloads/en/devicedoc/70138c.pdf#search=%27dsPIC30F3014%27 "dsPIC30F3014")

###I2CCONレジスタ

![register_map]({{ BASE_PATH }}/images/register_map.jpg)

 - **I2CCON<15> I2CEN**  

           [0]: I2Cモジュールを無効
           [1]: I2Cモジュールを有効

   I2Cモジュールを有効にした際マスタ、スレーブ両方有効になります。

 - **I2CCON<13> I2CSIDL**  

           [0]: アイドル時動作継続
           [1]: アイドル時動作中止

 - **I2CCON<12> SCLREL**  
   スレーブとして動作時

           [0]: SCLをLowに保持する
           [1]: SCLを自由に動作させる

 - **I2CCON<11> IPMIEN**  

           [0]: Intelligent peripheral management interface(IPMI)を無効
           [1]: IPMIを有効

 - **I2CCON<10> A10M**  

           [0]: I2CADDは7ビットアドレスモード
           [1]: 10ビットアドレスモード

 - **I2CCON<9> DISSLW**  

           [0]: スルーレート制御を有効
           [1]: スルーレート制御を無効

 - **I2CCON<8> SMEN**  

           [0]: SMBus入力のしきい値を無効
           [1]: I/Oピンを使用可能にしSMBusの使用に準拠したしきい値が設定される。

 - **I2CCON<7> GCEN**  
   スレーブとして動作時

           [0]: 一斉呼び出しアドレスを無効化
           [1]: 一斉呼び出しアドレスがI2CRSRで受信された場合に割り込みを有効化

 - **I2CCON<6> STREN**  
   スレーブとして動作時

           [0]: クロック延長のソフトと受信を無効化
           [1]: クロック延長のソフトと受信を有効化

 - **I2CCON<5> ACKDT**  
   マスタとして動作時

           [0]: 応答としてNACKを送信
           [1]: 応答としてACKを送信

 - **I2CCON<4> ACKEN**  
   マスタとして動作時

           [0]: 応答シーケンスを使わない
           [1]: SDAとSCLピン上でシーケンスの確認応答を開始し、ADKDTデータビットを送信

 - **I2CCON<3> RCEN**  
   マスタとして動作時

           [0]: 受信シーケンスを有効化しない
           [1]: I2Cの受信モードを有効化

 - **I2CCON<2> PEN**  
   マスタとして動作時

           [0]: ストップコンディションを無効
           [1]: SDAとSCLを使ってストップコンディションを送信

 - **I2CCON<1> RSEN**  
   マスタとして動作時

           [0]: Repeated START条件を有効化しない
           [1]: SDAピン及びSCLピンでRepeated START条件を開始

 - **I2CCON<0> SEN**  
   マスタとして動作時

           [0]: スタートコンディションを送信しない
           [1]: スタートコンディションを送信する

###I2CSTATレジスタ

 - **I2CSTAT<15> ACKSTAT**  
   マスタとして動作時

           [0]: アクノリッジをスレーブから受信
           [1]: アクノリッジをスレーブから受信待ち

 - **I2CSTAT<14> TRSTAT**  
   マスタとして動作時

           [0]: 送信終了
           [1]: (8ビット+ACK)の送信中

 - **I2CSTAT<10> BCL**  

           [0]: バス衝突なし
           [1]: バス衝突を検出

 - **I2CSTAT<9> GCSTAT**  
   ジェネラルコールアドレスについて

           [0]: 受信していない
           [1]: 受信した

 - **I2CSTAT<8> ADD10**  
   
           [0]: 10ビットのアドレス不一致
           [1]: 10ビットのアドレス一致

 - **I2CSTAT<7> IWCOL**  
           [0]: 書き込み衝突なし
           [1]: I2Cがビジー状態(I2CTRNに書き込みましょう)

 - **I2CSTAT<6> I2COV**  

           [0]: オーバーフローなし
           [1]: I2CRCVレジスタが前のバイトを保持している間にバイトを受信した

 - **I2CSTAT<5> D_A**  
   スレーブとして動作時、最後に受信したバイトは

           [0]: デバイスアドレス
           [1]: データ

 - **I2CSTAT<4> P**  

           [0]: ストップビットが最後に検出されなかった
           [1]: ストップビットが最後に検出されていた

 - **I2CSTAT<3> S**   
   最後にスタートコンディションが

           [0]: 検出されなかった
           [1]: 検出された(もしかしたら反復スタートコンディション)

 - **I2CSTAT<2> R_W**  
   スレーブとして動作時

           [0]: データ書き込み中
           [1]: データ読み込み中

 - **I2CSTAT<1> RBF**  
   
           [0]: 受信未完了(I2CRCVが空)
           [1]: 受信完了(I2CRCVがいっぱい)

 - **I2CSTAT<0> TBF**  

           [0]: 送信完了(I2CTRNが空)
           [1]: 送信中  (I2CTRNがいっぱい)

   検証した結果、TBFとTRSTATは同時に0になります

###I2CBRG

通信速度の設定を行います。

![I2CBRG]({{ BASE_PATH }}/images/I2CBRG.jpg)

400kHzバスで動作している場合、I2Cの仕様上スルーレート制御が必要となります。

I2CCON<9>がクリアされると、スルーレート制御はアクティブになります。

(注) その他のバス速度では、スルーレート制御は不要でDISSLWを設定する必要があります

**********

##I2C入出力設定に関する注意点

バスオペレーションに使用されるクロック(SCL)ピンとデータ(SDA)ピンはモジュールソフトウェアによってピンのポート入出力を制御しなくても、モジュールがポートの状態と方向をオーバーライドします。

**********

##電圧差に関する注意点

    dsPIC30F3014 -> 5V駆動
    IMU3000      -> 3.3V駆動

駆動電圧が異なるため[I2Cバス用双方向電圧レベル変換モジュール](http://akizukidenshi.com/catalog/g/gM-05452 "PCA9306")を使いましょう。

![Level]({{ BASE_PATH }}/images/LevelConv.jpg)

**********

以上でI2Cの基礎に関する解説を終了します。

次回は実際にジャイロセンサを動かすサンプルプログラムを紹介する予定です。

