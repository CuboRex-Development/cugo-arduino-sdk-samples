# CuGoSDKSampleコマンドリファレンス
クローラロボット開発プラットフォームの初心者向けサンプルコードはこちらをご参照ください</br>
https://github.com/CuboRex-Development/cugo-arduino-beginner-programming

旧製品のArduinoKitの方はこちらをご覧ください。</br>
https://github.com/CuboRex-Development/cugo-sdk-samples/tree/uno

<!-- クイックリファレンス はじめ-->

<!--
## 目次
- [1. はじめに](#1-はじめに)
- [2. 使い方](#2-使い方)
  - [2-1. 自動走行モード関数例](#2-1-自動走行モード関数例)
- [3. 各種関数](#3-各種関数)
  - [3-1. 自動走行関連](#3-1-自動走行関連)
    - [cugo\_move\_forward](#cugo_move_forward)
    - [cugo\_turn\_clockwise](#cugo_turn_clockwise)
    - [cugo\_turn\_counterclockwise](#cugo_turn_counterclockwise)
    - [cugo\_curve\_theta\_raw](#cugo_curve_theta_raw)
    - [cugo\_curve\_distance\_raw](#cugo_curve_distance_raw)
    - [cugo\_stop](#cugo_stop)
    - [cugo\_wait](#cugo_wait)
  - [3-2. 各種センサ取得関連](#3-2-各種センサ取得関連)
    - [cugo\_check\_odometer](#cugo_check_odometer)
  - [3-3. その他](#3-3-その他)
    - [cugo\_setup](#cugo_init)→★initに変更
    - [cugo\_motor\_direct\_instructions](#cugo_rpm_direct_instructions)
-->
<a id="anchor1"></a>

## 1. はじめに
これはクローラロボット開発プラットフォームをより発展的に使うために、さまざまな便利関数をまとめたサンプルコードです。CuGoを動かす各種関数等を説明します。

![V4メイン](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/e1b76ade-498c-49db-9c62-2013c0201fa4)

## 2. 準備
クローラロボット開発プラットフォームの利用開始までの手順を説明します。

### 2-1. Arduino IDEのインストール
1. 公式ページ( https://www.arduino.cc/en/software )へ移動
2. DOWNLOAD OPTIONSからご自身のOSのバージョンを選択
3. JUST DOWNLOADかCONTRIBUTE & DOWNLOADを選択
4. ダウンロードされたらファイルを実行して指示に従いインストール
### 2-2. 学習用ソースコードダウンロード
1. Codeボタンを選択し、Download ZIPをクリックしてダウンロード
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/ea21aa32-1b2e-4d48-852d-678ad300485b)

2. ダウンロードしたファイルを解凍
3. CugoSDKSample.inoをダブルクリックし、Arduino IDEを起動

### 2-3. Rasberry Pi Picoの初期設定
Arduino IDE でRaspberryPiPicoに書き込む場合、IDEにRaspberryPiPicoのボード情報をあらかじめ取得する必要があります。</br>

Arduino IDE バージョン2系（最新版）の場合

1. ファイル ＞ 環境設定を選択</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/f08cbb59-36f2-4ba0-80a3-889a7c337e0f)

2. 追加のボードマネージャのURLに以下のURLを入力し、OKを押します。右のウィンドウボタンをクリックすると入力できるようになります。</br>
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/fc20fa37-3747-4060-8da3-9ae169b9df93)

4. ツール ＞ ボード ＞ ボードマネージャ…を選択
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/320f1434-1135-4788-8b6e-df6194c24781)

5. "pico"で検索し、”Raspberry Pi Pico/RP2040”を見つけます。”INSTALL”ボタンを押します（すでに入っている場合はUPDATEボタンを押して最新にします）。
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/98a99647-b980-4e2b-aec4-b91b3419649b)
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/f5135740-f1cb-436a-8d63-f68a6cd3122f)

6. スケッチ ＞ ライブラリをインクルード… ＞ ライブラリを管理…を選択
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/1e1775bd-3399-4f40-82d8-8d5590d21918)

7. RPi_Pico_TimerInterrupt.hで検索し、”RPI_PICO_TimerInterrupt”を見つけます。”INSTALL”ボタンを押します（すでに入っている場合はUPDATEボタンを押して最新にします）。
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/1216aeee-82ef-4b6a-8cab-25935b4cab82)
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/d236d67f-2156-4f86-a9bd-0e3b68070bc9)




Arduino IDE バージョン1.8.19（レガシー）の場合

<details>
1. ファイル ＞ 環境設定を選択</br>

![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/f86d603a-0910-4281-a71f-8a6cde1fee40)


2. 追加のボードマネージャのURLに以下のURLを入力し、OKを押します。右のウィンドウボタンをクリックすると入力できるようになります。</br>
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/0b957362-a55b-4591-a8e7-7bb681eed5a0)

4. ツール ＞ ボード ＞ ボードマネージャ…を選択
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/ac1f3c62-d372-4bab-ae1a-3f9090a69e10)


5. "pico"で検索し、”Raspberry Pi Pico/RP2040”を見つけます。”インストール”ボタンを押します（すでに入っている場合は"更新"ボタンを押して最新にします）。
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/6341088e-1532-4c35-85d6-fcca6491ac46)
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/be7e4e8c-fc26-4fa0-9182-6f0392029942)


6. スケッチ ＞ ライブラリをインクルード… ＞ ライブラリを管理…を選択
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/cf2bb2d8-17b1-4421-8b2a-f86456e95f83)


7. RPi_Pico_TimerInterrupt.hで検索し、”RPI_PICO_TimerInterrupt”を見つけます。”インストール”ボタンを押します（すでに入っている場合は"更新"ボタンを押して最新にします）。
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/af765969-7bbd-4d24-8ce6-1c5496885bd2)


</details>

### 2-4. 制御パラメータの確認
クローラロボット開発プラットフォームには、V3iモデルとV4モデルがあります。各モデルごとに制御パラメータが異なる場所がありますので、確認をしてください。</br>

Arduino IDE上部タブから”CugoSDK.h”を選択
![image](https://github.com/CuboRex-Development/cugo-sdk-samples/assets/27883660/52c52b9e-edfc-42bd-aee7-b37fe62b2a81)


下記パラメータを使用しているモデルに合わせます。
https://github.com/CuboRex-Development/cugo-sdk-samples/blob/495630e56496518c560e93d79081504991ed0dac/CugoSDKSample/CugoSDK.h#L10-L24

プログラムが下の図と同じになるように必要に応じてプログラムを書き換え、ご自身のモデルのパラメータが反映されるように調整してください。</br>

V4の場合</br>
”CugoSDK.h”は、サンプルプログラムから変更する必要はありません。</br>
![image](https://github.com/CuboRex-Development/cugo-sdk-samples/assets/27883660/52f4ecd3-34ab-437b-9f02-a754207bd8a9)

V3iの場合</br>
”CugoSDK.h”の12行目から17行目の行頭に`//`を追記し、19行目から24行目の行頭の`//`を削除します。</br>
![image](https://github.com/CuboRex-Development/cugo-sdk-samples/assets/27883660/2d2c337b-0508-4adc-baf2-e9bfe8a6111d)


### 2-5. Rasberry Pi Picoへの書き込み

1. CugoBeginnerProgramming.inoがArduino IDEで開かれていることを確認
2. USBケーブルでパソコンとRaspberry Pi Picoを接続。PCに認識されないときは基板にある”BOOTSEL”ボタンを押しながらPCに挿してください。</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/483c2a4e-8850-4924-90de-a17b09dd775f)

4. ツール ＞ ボード ＞ Raspberry Pi Pico/RP2040 ＞ Rasberry Pi Pico を選択
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/db962aa7-1deb-4125-a7f9-eec2b2707433)

6. ツール ＞ ポート からRasberry Pi Picoのポートを選択します。ポートはUSBを接続する前と後を比較して増えたものがRaspberry Pi Picoなので、それを選択します。</br>
USBを接続する前↓</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/02be584c-bd9d-4c96-a9dc-0bca2dbb5e5a)

USBを接続した後↓</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/50e47ad7-b85d-4dd5-82ba-f441ba3da83d)

8. 矢印ボタン " → "を選択し、マイコンボードへ書き込むを実行</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/a7bd9db9-d3d0-4221-963a-2bff952eb833)

9. ボードへの書き込みが完了しましたの記載があれば書き込み完了です。

### 2-6. LD-2のコマンドモードを有効化
クローラロボット開発プラットフォームでは、安全のため、出荷時にプログラム動作する、コマンドモードを無効化しています。</br>
以下の図に従って、電源が切れていることを確認し、DIPスイッチの2番をON側に倒してください。大変小さなスイッチですので、つまようじなどを用意して操作してください。</br>
詳細は、取扱説明書をご覧ください。</br>
https://drive.google.com/drive/folders/18MQUVMLYS_4JgkeGd2v7dVHmdmFaMaZc?usp=drive_link

![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/fca21c7a-01e0-45cb-9bff-3d91ef784300)



<a id="anchor2"></a>

## 3. 使い方

- CuGoSDKSampleにて自作関数を利用する場合は`loop`内の `//ここから自動走行モードの記述`から `//ここまで自動走行モードの記述`までに記載してください。

```c
void loop() {

  //プロポ入力によりcugo_run_modeが変化
  if(cugo_runmode == CUGO_RC_MODE){//ラジコンモード
    ld2_set_control_mode(CUGO_RC_MODE);
    cugo_wait(100);
  }else if(cugo_runmode == CUGO_CMD_MODE){//自動走行モード
    //ここから自動走行モードの記述  


    //ここまで自動走行モードの記述
    
    ld2_set_control_mode(CUGO_RC_MODE); 
    cugo_wait(1000);
    //自動走行モードを1回のloopで終了する場合のみ記載、不要の場合コメントアウト
    
  }
}
```
<a id="anchor2-1"></a>

### 3-1. 自動走行モード関数例

- 正方形に移動させたい場合は下記コードを参考にしてください

```c
void loop() {

  //プロポ入力によりcugo_run_modeが変化
  if(cugo_runmode == CUGO_RC_MODE){//ラジコンモード
    ld2_set_control_mode(CUGO_RC_MODE);
    cugo_wait(100);
  }else if(cugo_runmode == CUGO_CMD_MODE){//自動走行モード
    //ここから自動走行モードの記述  
      //サンプルコード記載
                
        Serial.println(F("自動走行モード開始"));  
        cugo_wait(1000);
        
        Serial.println(F("1.0mの正方形移動の実施"));
        cugo_move_forward(1.0);
        cugo_wait(1000);
        cugo_turn_clockwise(90);
        cugo_wait(1000);
        cugo_move_forward(1.0);
        cugo_wait(1000);
        cugo_turn_clockwise(90);
        cugo_wait(1000);
        cugo_move_forward(1.0);
        cugo_wait(1000);
        cugo_turn_clockwise(90);
        cugo_wait(1000);
        cugo_move_forward(1.0);
        cugo_wait(1000);
        cugo_turn_clockwise(90);
        cugo_wait(1000);
        
        Serial.println(F("自動走行モード終了"));
        
      //サンプルコード終了

    //ここまで自動走行モードの記述
    
    ld2_set_control_mode(CUGO_RC_MODE); 
    cugo_wait(1000);
    //自動走行モードを1回のloopで終了する場合のみ記載、不要の場合コメントアウト
    
  }
}
```
- S字に移動させたい場合は下記コードを参考にしてください

```c
void loop() {

  //プロポ入力によりcugo_run_modeが変化
  if(cugo_runmode == CUGO_RC_MODE){//ラジコンモード
    ld2_set_control_mode(CUGO_RC_MODE);
    cugo_wait(100);
  }else if(cugo_runmode == CUGO_CMD_MODE){//自動走行モード
    //ここから自動走行モードの記述  
    Serial.println(F("自動走行モード開始"));  
    cugo_wait(1000);

    Serial.println(F("半径1.0mのS字移動"));
    cugo_curve_theta_raw(1.0,180,90);
    cugo_wait(1000);  
    cugo_curve_theta_raw(-1.0,180,90);
    cugo_wait(1000);  

    Serial.println(F("自動走行モード終了")); 

    //ここまで自動走行モードの記述
    
    ld2_set_control_mode(CUGO_RC_MODE); 
    cugo_wait(1000);
    //自動走行モードを1回のloopで終了する場合のみ記載、不要の場合コメントアウト
    
  }
}
```
- その他、関数の使い方の例は`CugoSDK.cpp`内の`cugo_test`を参考にしてください。

<a id="anchor3"></a>

## 4. 各種関数

<a id="anchor3-1"></a>

### 4-1. 自動走行関連
<a id="anchor3-1-1"></a>

### cugo_move_forward
- 【説明】
  - CuGoが前進後進する関数です。
  - `cugo_move_forward_raw`は位置制御を実施していないため、上限速度まで急峻に到達し、目標距離を超えると急停止します。
- 【構文】
  -  `cugo_move_forward(float target_distance)`
  -  `cugo_move_forward(float target_distance,float target_rpm)`
  - `cugo_move_forward_raw(float target_distance,float target_rpm)`
- 【パラメータ】
  - `target_distance` ：目標距離　単位はm
  - `target_rpm` ：上限速度　単位はrpm
- 【戻り値】
  - なし

<a id="anchor3-1-2"></a>

### cugo_turn_clockwise
- 【説明】
  - CuGoが時計回りに回転する関数です。
  - `cugo_turn_clockwise_raw`は位置制御を実施していないため、上限速度まで急峻に到達し、目標距離を超えると急停止します。
- 【構文】
  - `cugo_turn_clockwise(float target_degree)`
  - `cugo_turn_clockwise(float target_degree,float target_rpm)`
  - `cugo_turn_clockwise_raw(float target_degree,float target_rpm)`
- 【パラメータ】
  - `target_degree` ：目標距離　単位は度
  - `target_rpm` ：上限速度　単位はrpm
- 【戻り値】
  - なし

<a id="anchor3-1-3"></a>

### cugo_turn_counterclockwise
- 【説明】
  - CuGoが反時計回りに回転する関数です。
  - `cugo_turn_counterclockwise_raw`は位置制御を実施していないため、上限速度まで急峻に到達し、目標距離を超えると急停止します。
- 【構文】
  - `cugo_turn_counterclockwise(float target_degree)`
  - `cugo_turn_counterclockwise(float target_degree,float target_rpm)`
  - `cugo_turn_counterclockwise_raw(float target_degree,float target_rpm)`
- 【パラメータ】
  - `target_degree` ：目標距離　単位は度
  - `target_rpm` ：上限速度　単位はrpm
- 【戻り値】
  - なし

<a id="anchor3-1-4"></a>

### cugo_curve_theta_raw

- 【説明】
  - CuGoが指定した半径を指定した角度まで円軌道する関数です。
  - `cugo_turn_counterclockwise_raw`は位置制御を実施していないため、上限速度まで急峻に到達し、目標距離を超えると急停止します。
- 【構文】
  - `cugo_curve_theta_raw(float target_radius,float target_degree,float target_rpm)`
- 【パラメータ】
  - `target_radius` ：円軌道半径　単位はm
    - 円軌道半径は指定した値が正の場合、原点はcugoの進行方向左側になります
  - `target_degree` ：円軌道角度　単位は度
  - `target_rpm` ：上限速度　単位はrpm
- 【戻り値】
  - なし
 
<a id="anchor3-1-5"></a>

### cugo_curve_distance_raw
- 【説明】
  - CuGoが指定した半径を指定した移動距離まで円軌道する関数です。
  - `cugo_turn_counterclockwise_raw`は位置制御を実施していないため、上限速度まで急峻に到達し、目標距離を超えると急停止します。
- 【構文】
  - `cugo_curve_distance_raw(float target_radius,float target_disttance,float target_rpm)`  
- 【パラメータ】
  - `target_radius` ：円軌道半径　単位はm 
    - 円軌道半径は指定した値が正の場合、原点はCuGoの進行方向左側になります
  - `target_disttance` ：円軌道距離　単位はm
  - `target_rpm` ：上限速度　単位はrpm
- 【戻り値】
  - なし

<a id="anchor3-1-6"></a>

### cugo_stop
- 【説明】
  - CuGoが停止する関数です。
- 【構文】
  - `cugo_stop()`  
- 【パラメータ】
  - なし
- 【戻り値】
  - なし
<a id="anchor3-1-7"></a>

### cugo_wait
- 【説明】
  - CugoSDKSampleでは割り込み処理を利用してモード遷移をしているため、**delay関数を使用しないことを強く推奨します。**
  - `cugo_wait`はdelay関数と同等の役割を果たす関数になります。
  - `cugo_wait`はCugoSDKSample内で利用する停止関数です。
  - `cugo_wait`の計測時間の上限は約70分です。７０分以上の計測には`cugo_long_wait`を利用してください。
  - PICO内の水晶子を利用しているため時間は正確でない場合があります。

- 【構文】
  - `cugo_wait(unsigned long long int wait_ms)`  
  - `cugo_long_wait(unsigned long long int wait_seconds)`  
- 【パラメータ】
  - `wait_ms` ：停止時間　単位はミリ秒
  - `wait_sedonds` ：停止時間　単位は秒
- 【戻り値】
  - なし

<a id="anchor3-2"></a>

### 4-2. 各種センサ取得関連

<a id="anchor3-2-1"></a>

### cugo_check_odometer
- 【説明】
  - CuGoのオドメトリを取得します。
- 【構文】
  - `cugo_check_odometer(int check_number)`
- 【パラメータ】
    - `check_number` :以下から選択
    - `CUGO_ODO_X`  , `CUGO_ODO_Y`  , `CUGO_ODO_THETA`, `CUGO_ODO_DEGREE`
    - 上記の定数はdefineによりそれぞれ0,1,2,3の数値に対応しています。  
- 【戻り値】
  - float型  

<a id="anchor3-3"></a>

### 4-3. その他

<a id="anchor3-3-1"></a>

### cugo_init
- 【説明】
  - CuGoを利用するための初期設定関数です。setup関数内にあります。
- 【構文】
  - `cugo_init`

<a id="anchor3-3-3"></a>

### cugo_rpm_direct_instructions
- 【説明】
  - モーターへプロポ入力します。
  - モーターを停止させたい場合は左右のパラメータに`0`を入力します。
- 【構文】
  - `cugo_rpm_direct_instructions(float left, float right)` 
- 【パラメータ】
  - `left` 左モーターへの入力値(RPM)
  - `right` 右モーターへの入力値(RPM)
- 【戻り値】
  - なし
<!-- ★チェック-->


<!-- クイックリファレンス ここまで-->  
