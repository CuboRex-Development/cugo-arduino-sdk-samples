/****************************************************************************
 * CugoArduinoMiddleUserProgramming                                           *
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
/*****************CugoArduinoMiddleUserProgramming ver1.00*********************/
/****************************************************************************
 * CugoArduinoMiddleUserProgrammingをご利用される方へ
 *  詳細はクイックリファレンス.mdをご確認ください。
 ****************************************************************************/
#include <Arduino.h>
#include "CugoArduinoMiddle.h"

//利用するMotorControllerクラスの宣言
MotorController cugo_motor_controllers[CUGO_MOTOR_NUM];
//初期化関数
void(*resetFunc)(void) = 0;

void setup()
{
  Serial.begin(115200);
  cugo_setup();
}

void loop()
{
  cugo_check_mode_change(cugo_motor_controllers);//ラジコンモードと自動走行モードの切り替わり確認
  if(cugoRunMode == CUGO_RC_MODE){//ラジコンモード
    cugo_rcmode(cugoRcTime,cugo_motor_controllers);   
  }else if(cugoRunMode == CUGO_ARDUINO_MODE){//自動走行モード
    //ここから自動走行モードの記述
  
      //サンプルコード記載
        Serial.println(F("自動走行モード開始"));  
        //cugo_wait(1000);
        
        Serial.println(F("1.0mの正方形移動の実施"));
        cugo_move_forward(1.0,cugo_motor_controllers);
        cugo_wait(1000);
        cugo_turn_clockwise(90,cugo_motor_controllers);
        cugo_wait(1000);
        cugo_move_forward(1.0,cugo_motor_controllers);
        cugo_wait(1000);
        cugo_turn_clockwise(90,cugo_motor_controllers);
        cugo_wait(1000);
        cugo_move_forward(1.0,cugo_motor_controllers);
        cugo_wait(1000);
        cugo_turn_clockwise(90,cugo_motor_controllers);
        cugo_wait(1000);
        cugo_move_forward(1.0,cugo_motor_controllers);
        cugo_wait(1000);
        cugo_turn_clockwise(90,cugo_motor_controllers);
        cugo_wait(1000);
        
        Serial.println(F("自動走行モード終了"));
        
      //サンプルコード終了

    cugoRunMode = CUGO_RC_MODE; //自動走行モードを1回のloopで終了してラジコンモードへ移行したい場合はCUGO_RC_MODEを入力し、自動走行モードを繰り返し実行したい場合はCUGO_ARDUINO_MODEを入力

    //ここまで自動走行モードの記述  
  }
}

//割り込み処理
ISR(PCINT2_vect)
{ 
  //PROPO Achの割り込み
  if (CUGO_OLD_PWM_IN_PIN0_VALUE != digitalRead(CUGO_PWM_IN_PIN0))
  {
    if (LOW == CUGO_OLD_PWM_IN_PIN0_VALUE)
    { // 立ち上がり時の処理
      CUGO_PIN_UP(0);
    }
    else
    { // 立下り時の処理
      CUGO_PIN_DOWN(0);
    }
    CUGO_OLD_PWM_IN_PIN0_VALUE = CUGO_OLD_PWM_IN_PIN0_VALUE ? LOW : HIGH;
  }

  //PROPO Bchの割り込み
  if (CUGO_OLD_PWM_IN_PIN1_VALUE != digitalRead(CUGO_PWM_IN_PIN1))
  {  
    if (LOW == CUGO_OLD_PWM_IN_PIN1_VALUE)
    { // 立ち上がり時の処理
      CUGO_PIN_UP(1);
    }
    else
    { // 立下り時の処理
      CUGO_PIN_DOWN(1);//たおした瞬間にリセットになっているかを確認
      if (CUGO_ARDUINO_MODE_IN < cugo_time[1] && CUGO_PROPO_MAX > cugo_time[1])
      { //KOPPROのBchを右に倒すと自動走行モードのフラグに切替　※cugo_check_mode_change以降で動作切替
        cugoRunMode = CUGO_ARDUINO_MODE;
      } 
      if (CUGO_ARDUINO_MODE_OUT > cugo_time[1] && CUGO_PROPO_MIN < cugo_time[1])
      { //KOPPROのBchを左に倒すとArduinoのリセット
        resetFunc();
      }       
    }
    CUGO_OLD_PWM_IN_PIN1_VALUE = CUGO_OLD_PWM_IN_PIN1_VALUE ? LOW : HIGH;
  }

  //PROPO Cchの割り込み
  if (CUGO_OLD_PWM_IN_PIN2_VALUE != digitalRead(CUGO_PWM_IN_PIN2))
  {
    if (LOW == CUGO_OLD_PWM_IN_PIN2_VALUE)
    { // 立ち上がり時の処理
      CUGO_PIN_UP(2);
    }
    else
    { // 立下り時の処理
      CUGO_PIN_DOWN(2);
    }
    CUGO_OLD_PWM_IN_PIN2_VALUE = CUGO_OLD_PWM_IN_PIN2_VALUE ? LOW : HIGH;
  }

  //ボタン操作の割り込み
  if (CUGO_OLD_CMD_BUTTON_VALUE != digitalRead(CUGO_CMD_BUTTON_PIN)){
    if (LOW == CUGO_OLD_CMD_BUTTON_VALUE){ // 立ち上がり時の処理
      cugo_button_check = false;
    }
    else{ // 立下り時の処理
      CUGO_PIN_DOWN(3);
      CUGO_PIN_UP(3);
      cugo_button_check = true;
      if (CUGO_BUTTON_CHECK_BORDER < cugo_time[3] ){ 
          cugo_button_count++;
      }      
    }
    CUGO_OLD_CMD_BUTTON_VALUE = CUGO_OLD_CMD_BUTTON_VALUE ? LOW : HIGH;
  }    
}

//CUGOのセットアップ関連 
void cugo_setup()
{ //初期化実行
  cugo_init();
  // LEFTインスタンス有効化
  cugo_motor_controllers[CUGO_MOTOR_LEFT] = MotorController(CUGO_PIN_ENCODER_L_A, CUGO_PIN_ENCODER_L_B, CUGO_PIN_MOTOR_L, 2048, 600, 100, CUGO_L_LPF, CUGO_L_KP, CUGO_L_KI, CUGO_L_KD, CUGO_L_reverse);
  // RIGHTインスタンス有効化
  cugo_motor_controllers[CUGO_MOTOR_RIGHT] = MotorController(CUGO_PIN_ENCODER_R_A, CUGO_PIN_ENCODER_R_B, CUGO_PIN_MOTOR_R, 2048, 600, 100, CUGO_R_LPF, CUGO_R_KP, CUGO_R_KI, CUGO_R_KD, CUGO_R_reverse);
  // エンコーダカウンタは純正のハードウェア割り込みピンを使用
  attachInterrupt(digitalPinToInterrupt(CUGO_PIN_ENCODER_L_A), cugoLeftEncHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(CUGO_PIN_ENCODER_R_A), cugoRightEncHandler, RISING);
  // 初期値でモータ指示。起動時に停止を入力しないと保護機能が働き、回りません。
  cugo_motor_direct_instructions(1500, 1500,cugo_motor_controllers); //直接停止命令を出す
  delay(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。
}
//エンコーダー割り込み時の実行処理 
void cugoLeftEncHandler(){
  cugo_motor_controllers[CUGO_MOTOR_LEFT].updateEnc();
}
//エンコーダー割り込み時の実行処理
void cugoRightEncHandler(){
  cugo_motor_controllers[CUGO_MOTOR_RIGHT].updateEnc();
}
