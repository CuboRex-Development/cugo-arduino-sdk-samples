/****************************************************************************
 * CugoArduinoBeginnerProgramming                                           *
 *                                                                          *
 * Copyright [2022] [CuboRex.Inc]                                           *
 * Licensed under the Apache License, Version 2.0 (the “License”);          *
 * you may not use this file except in compliance with the License.         *
 * You may obtain a copy of the License at                                  *
 *                                                                          * 
 * http://www.apache.org/licenses/LICENSE-2.0                               *
 *                                                                          *
 * Unless required by applicable law or agreed to in writing, software      *
 * distributed under the License is distributed on an “AS IS” BASIS,        *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 * See the License for the specific language governing permissions and      *
 * limitations under the License.                                           *
 ****************************************************************************/
/*****************CugoArduinoBeginnerProgramming ver1.00*********************/
/****************************************************************************
 * CugoArduinoBeginnerProgrammingをご利用される方へ
 *  スクロールして「Arduino学習用プログラミングはここから」からご確認ください。
 *  詳細はREADME.mdをご確認ください。
 ****************************************************************************/
#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include "CugoArduinoMiddle.h"
#include "MotorController.h"

//プロトタイプ宣言
//void CMD_EXECUTE();//★消す
void(*resetFunc)(void) = 0;


//利用するモーター数の宣言
MotorController cugo_motor_controllers[MOTOR_NUM];

//初期設定
void setup()
{
  Serial.begin(115200);
  cugo_setup_middle();

}

//loop内を繰り返し実行
void loop()
{
  cugo_check_mode_change();
  if (cugoRunMode == CUGO_RC_MODE){
    cugo_rcmode(cugoRcTime,cugo_motor_controllers);//RC（ラジコン）操作
  }
  if (cugoRunMode == CUGO_ARDUINO_MODE){//ここから自動走行モードの記述 //★Arduinomodeではなくself-driveが良い？
  //サンプルコード記載
  
  }
  
  //current_time = micros();  // オーバーフローまで約40分
  /*
  if (current_time - prev_time_10ms > 10000) 
  {
    job_10ms();
    prev_time_10ms = current_time;
  }
  //display_detail(cugo_motor_controllers);//必要に応じてCugoArduinoModeの5～8行目を変更
  */
}

//割り込み処理
ISR(PCINT2_vect)
{ 
  
  if (OLD_PWM_IN_PIN0_VALUE != digitalRead(PWM_IN_PIN0))
  {
    if (LOW == OLD_PWM_IN_PIN0_VALUE)
    { // 立ち上がり時の処理
      PIN_UP(0);
    }
    else
    { // 立下り時の処理
      PIN_DOWN(0);
    }
    OLD_PWM_IN_PIN0_VALUE = OLD_PWM_IN_PIN0_VALUE ? LOW : HIGH;
  }
  if (OLD_PWM_IN_PIN1_VALUE != digitalRead(PWM_IN_PIN1))
  {  
    if (LOW == OLD_PWM_IN_PIN1_VALUE)
    { // 立ち上がり時の処理
      PIN_UP(1);
    }
    else
    { // 立下り時の処理
      PIN_DOWN(1);
      if (CUGO_ARDUINO_MODE_IN < time[1])
      { //KOPPROのBchを右に倒すとArduinoリセット
      cugo_Bch_flag = false;
      resetFunc();
      }else if (CUGO_ARDUINO_MODE_OUT > time[1])
      { //KOPPROのBchを左に倒すとモードフラグの変更
        if(cugo_Bch_flag){
          if (cugoRunMode == CUGO_RC_MODE){
            cugoRunMode = CUGO_ARDUINO_MODE;
          }else if (cugoRunMode == CUGO_ARDUINO_MODE){
            cugoRunMode = CUGO_RC_MODE;  
          }
          cugo_Bch_flag = false;
        }
      }else
      {
      cugo_Bch_flag = true;
      }       
    }
    OLD_PWM_IN_PIN1_VALUE = OLD_PWM_IN_PIN1_VALUE ? LOW : HIGH;
  }
  if (OLD_PWM_IN_PIN2_VALUE != digitalRead(PWM_IN_PIN2))
  {
    if (LOW == OLD_PWM_IN_PIN2_VALUE)
    { // 立ち上がり時の処理
      PIN_UP(2);
    }
    else
    { // 立下り時の処理
      PIN_DOWN(2);
    }
    OLD_PWM_IN_PIN2_VALUE = OLD_PWM_IN_PIN2_VALUE ? LOW : HIGH;
  }

  if (OLD_CMD_BUTTON_VALUE != digitalRead(CMD_BUTTON_PIN))
  {
    if (LOW == OLD_CMD_BUTTON_VALUE)
    { // 立ち上がり時の処理
      PIN_UP(3);
      button_check = true;//★ボタンはこれで判定でよいか？
    }
    else
    { // 立下り時の処理
      PIN_DOWN(3);
      button_check = false;
    }
    OLD_CMD_BUTTON_VALUE = OLD_CMD_BUTTON_VALUE ? LOW : HIGH;
  }  
}

/*
//自動走行モード(arduino_mode)の実行
void arduino_mode()
{

  //CMD_EXECUTE();

}
*/

//割り込み時の実行処理関連 
void cugo_setup_middle()
{
  cugo_init_middle();

  // LEFTインスタンス有効化
  cugo_motor_controllers[MOTOR_LEFT] = MotorController(PIN_ENCODER_L_A, PIN_ENCODER_L_B, PIN_MOTOR_L, 2048, 600, 100, L_LPF, L_KP, L_KI, L_KD, L_reverse);
  // RIGHTインスタンス有効化
  cugo_motor_controllers[MOTOR_RIGHT] = MotorController(PIN_ENCODER_R_A, PIN_ENCODER_R_B, PIN_MOTOR_R, 2048, 600, 100, R_LPF, R_KP, R_KI, R_KD, R_reverse);

  // エンコーダカウンタは純正のハードウェア割り込みピンを使用
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_A), cugoLeftEncHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_A), cugoRightEncHandler, RISING);

  // 初期値でモータ指示。起動時に停止を入力しないと保護機能が働き、回りません。
  cugo_motor_direct_instructions(1500, 1500,cugo_motor_controllers); //直接停止命令を出す
  delay(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。
}

//割り込み時の実行処理関連 //setupであることを隠す
void cugoLeftEncHandler()
{
  cugo_motor_controllers[MOTOR_LEFT].updateEnc();
}

//割り込み時の実行処理関連　//setupであることを隠す
void cugoRightEncHandler()
{
  cugo_motor_controllers[MOTOR_RIGHT].updateEnc();
}

/***** Arduino学習用プログラミングはここから *****/
/* ヒント：使えるコマンドリスト */
/*
 * ★基本コマンド★
 *    button();          //ボタンの押し待ち     
 *    matsu(1000);       //1000ミリ秒待機       □詳細：入力した()内の数字ミリ秒だけ待機する。1000ミリ秒＝1秒。
 *    susumu(1.0);       //1.0m前に進む         □詳細：入力した()内の数字m 分だけ前に進む
 *    sagaru(1.0);       //1.0m後ろに進む       □詳細：入力した()内の数字m 分だけ後ろに進む
 *    migimawari45();    //45度右に回転する 
 *    migimawari90();    //90度右に回転する
 *    migimawari180();   //180度右に回転する
 *    hidarimawari45();  //45度右に回転する
 *    hidarimawari90();  //90度右に回転する
 *    hidarimawari180(); //180度右に回転する
 * 
 * ★★応用コマンド★★
 *    susumu(1.0,90);                //上限速度90rpmで1.0m前に進む         □詳細：()内に移動距離と上限速度を設定。上限速度は最大180rpmで距離が短いと上限速度に到達しない場合もある。
 *    sagaru(1.0,90);                //上限速度90rpmで1.0m後ろに進む      　□詳細：()内の設定はsusumu(1.0,90)と同様
 *    migimawari45(90);              //上限速度90rpmで45度右に回転する      □詳細：()内に上限速度を設定。上限速度は最大180rpmで距離が短いと上限速度に到達しない場合もある。
 *    migimawari90(90);              //上限速度90rpmで90度右に回転する      □詳細：()内の設定はmigimawari45(90)と同様
 *    migimawari180(90);             //上限速度90rpmで180度右に回転する     □詳細：()内の設定はmigimawari45(90)と同様
 *    hidarimawari45(90);            //上限速度90rpmで45度左に回転する      □詳細：()内の設定はmigimawari45(90)と同様
 *    hidarimawari90(90);            //上限速度90rpmで90度左に回転する      □詳細：()内の設定はmigimawari45(90)と同様
 *    hidarimawari180(90);           //上限速度90rpmで180度左に回転する     □詳細：()内の設定はmigimawari45(90)と同様
 *    turn_clockwise(60,90);         //上限速度90rpmで60度右に回転する      □詳細：()内に回転角度と上限速度を設定。上限速度は最大180rpmで距離が短いと上限速度に到達しない場合もある。
 *    turn_counter_clockwise(60,90); //上限速度90rpmで60度左に回転する      □詳細：()内の設定はturn_clockwise(60,90)と同様
*/

/*
//コマンドの実行
void CMD_EXECUTE()
{
  cmd_manager(cugo_motor_controllers);  // おまじない

  // ここから↓を改造していこう！

  //例：ボタンが押されたら1mの正方形を描く動き
  
  button();//ボタンが押されるまで待つ

  susumu(1.0);//1.0m 進む
  matsu(1000);//1000ミリ秒(1秒) 待つ
  migimawari90();//右回りに90度回転
  matsu(1000);//1000ミリ秒(1秒) 待つ
   
  susumu(1.0);
  matsu(1000);
  migimawari90();
  matsu(1000); 
  
  susumu(1.0);
  matsu(1000);
  migimawari90();
  matsu(1000); 
  
  susumu(1.0);
  matsu(1000);
  migimawari90();
  matsu(1000); 
  
  // ここから↑を改造していこう！

  cmd_end(cugo_motor_controllers);      // おまじない
}
*/
