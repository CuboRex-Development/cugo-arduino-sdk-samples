# CugoSDKSampleコマンドリファレンス

初心者向けサンプルコードはこちらをご参照ください  
https://github.com/CuboRex-Development/cugo-arduino-beginner-programming
<!-- クイックリファレンス はじめ-->

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
    - [cugo\_motor\_direct\_instructions](#cugo_motor_direct_instructions)

<a id="anchor1"></a>

# 1. はじめに
- CuGoSDKSampleに関するクイックリファレンスです。CuGoSDKSampleを利用してCuGoを動かす上での各種関数等を説明します。
- CuGoSDKSampleを動かすための事前準備は以下を参考にしてください
  - [クローラロボット開発プラットフォームアップグレード説明書]→★変更(https://github.com/CuboRex-Development/cugo-arduino-beginner-programming/blob/462fc9e3d52d90777aff0116ad4912cc0e54ffc5/manuals/ArduinoKit%E3%82%A2%E3%83%83%E3%83%95%E3%82%9A%E3%82%AF%E3%82%99%E3%83%AC%E3%83%BC%E3%83%88%E3%82%99%E8%AA%AC%E6%98%8E%E6%9B%B8.pdf)

★リンクではなくbeginnerの同じMDファイルを格納する　
  - [クローラロボット開発プラットフォームクイックスタートガイド](https://github.com/CuboRex-Development/cugo-arduino-beginner-programming/blob/main/README.md#cugoarduinokit%E3%82%AF%E3%82%A4%E3%83%83%E3%82%AF%E3%82%B9%E3%82%BF%E3%83%BC%E3%83%88%E3%82%AC%E3%82%A4%E3%83%89)
    - ただし、クローラロボット開発プラットフォームクイックスタートガイドに記載のあるコマンドはbeginner用のため利用できません。
    - CuGoSDKのコマンドは本ページの [3. 各種関数](#3-各種関数)
をご利用ください。


<a id="anchor2"></a>

# 2. 使い方

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

## 2-1. 自動走行モード関数例

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

# 3. 各種関数

<a id="anchor3-1"></a>

## 3-1. 自動走行関連
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

## 3-2. 各種センサ取得関連

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

## 3-3. その他

<a id="anchor3-3-1"></a>

### cugo_init
- 【説明】
  - CuGoを利用するための初期設定関数です。setup関数内にあります。
- 【構文】
  - `cugo_init`

<a id="anchor3-3-3"></a>

### cugo_motor_direct_instructions
- 【説明】
  - モーターへプロポ入力します。
  - モーターを停止させたい場合は左右のパラメータに`0`を入力します。
- 【構文】
  - `cugo_motor_direct_instructions(int left, int right)` 
- 【パラメータ】
  - `left` 左モーターへのプロポ入力値
  - `right` 右モーターへのプロポ入力値
- 【戻り値】
  - なし
<!-- ★チェック-->


<!-- クイックリファレンス ここまで-->  
