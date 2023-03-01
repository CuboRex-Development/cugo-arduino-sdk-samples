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
 * CugoArduinoMiddleUserProgrammingをご利用される方へ
 *  詳細はREADME.mdをご確認ください。
 ****************************************************************************/
#include <Arduino.h>
#include "CugoArduinoMiddle.h"


//利用するモーターの宣言
MotorController cugo_motor_controllers[MOTOR_NUM];

//プロトタイプ宣言
void(*resetFunc)(void) = 0;


// 距離センサをりようするサンプルプログラム用
#define PIN_SENSOR A3  //   距離センサ用PIN

//初期設定
void setup()
{
  Serial.begin(115200);
  cugo_setup();
}

//loop内を繰り返し実行
void loop()
{
  cugo_check_mode_change(cugo_motor_controllers);
  if (cugoRunMode == CUGO_RC_MODE){    
    cugo_rcmode(cugoRcTime,cugo_motor_controllers);//RC（ラジコン）操作   
    //Serial.println("L::" + String(cugo_motor_controllers[0].getTargetRpm()) + "," + String(cugo_motor_controllers[0].getRpm()) + "," +  String(cugo_motor_controllers[0].getCount()));
    //Serial.println("R::" + String(cugo_motor_controllers[1].getTargetRpm()) + "," + String(cugo_motor_controllers[1].getRpm()) + "," +  String(cugo_motor_controllers[1].getCount()));
    //Serial.println(F("#   cugo_motor_direct_instructions"));//確認用
    //cugo_motor_controllers[0].servo_.writeMicroseconds(0);
    //cugo_motor_controllers[1].servo_.writeMicroseconds(0);
  }
  if (cugoRunMode == CUGO_ARDUINO_MODE){//ここから自動走行モードの記述 //★Arduinomodeではなくself-driveが良い？
  //サンプルコード記載
    Serial.println("#自動走行モード開始");
    Serial.println("##Ach : " + String(cugo_check_a_channel_value()) + " ,Bch : " + String(cugo_check_b_channel_value()) + " ,Cch : " + String(cugo_check_c_channel_value()));
    //Serial.println("check");
    Serial.println("##Button_status : " + String(cugo_check_button()));
    //Serial.println("check");
    Serial.println("##Button_times : " + String(cugo_check_button_times()));
    Serial.println("##sensor : " + String(analogRead(PIN_SENSOR)));
    
    //Serial.println("check");
    if(cugo_check_button_times() > 5) {
      cugo_reset_button_times();   
    }
    Serial.println("##Button_time : " + String(cugo_button_press_time()));
    cugo_move_forward(1.0,cugo_motor_controllers);
    cugo_keep_stop_ms(2000,cugo_motor_controllers);
    //cugo_keep_speed_ms(100,cugo_motor_controllers);
//    cugo_keep_speed_ms(100,cugo_motor_controllers);
    //cugo__direct(1.0,1,cugo_motor_controllers); //1m 進むのに8秒進んでいる　speedは160くらい　1rpm指定?
    //cugo_go(-1.0,cugo_motor_controllers);
    //cugo_wait_ms(2000,cugo_motor_controllers);
    //cugo_go(-1.0,0.2,cugo_motor_controllers);
    //cugo_wait_ms(2000,cugo_motor_controllers);
    //cugo_turn_direct(90,20,cugo_motor_controllers);
    //cugo_wait_ms(2000,cugo_motor_controllers);
    //cugo_turn(-90,cugo_motor_controllers);
    //cugo_wait_ms(2000,cugo_motor_controllers);
    //cugo_turn(45,0.5,cugo_motor_controllers);
    
    
    cugo_stop(cugo_motor_controllers);
  /*
  cugo_wait_ms(100,cugo_motor_controllers);
  Serial.println("wait done!1");
  cugo_go(1.0,cugo_motor_controllers);
  cugo_wait_ms(100,cugo_motor_controllers);
  Serial.println("wait done!2");
  cugo_go(-1.0,50,cugo_motor_controllers);
  cugo_wait_ms(100,cugo_motor_controllers);
  Serial.println("wait done!3");
  cugo_go_direct(1.0,150,cugo_motor_controllers);
  cugo_wait_ms(100,cugo_motor_controllers);
  Serial.println("wait done!4");
  cugo_turn(90,cugo_motor_controllers);
  cugo_wait_ms(100,cugo_motor_controllers);
  Serial.println("wait done!5");
  cugo_turn(-90,20,cugo_motor_controllers);//単位はm,rpm
  cugo_wait_ms(100,cugo_motor_controllers);
  Serial.println("wait done!6");
  cugo_turn_direct(45,30,cugo_motor_controllers);//単位はm,rpm
  cugo_wait_ms(100,cugo_motor_controllers);
  Serial.println("wait done!7");
  Serial.println("各プロポ入力値");
  Serial.print(String(cugo_check_a_channel_value()));   
  Serial.print(String(cugo_check_b_channel_value()));   
  Serial.println(String(cugo_check_c_channel_value())); 
  Serial.println("各ボタン状態");    
  Serial.print(String(cugo_check_button())); //現状の押されているか
  Serial.print(String(cugo_check_button_times())); //現状の押された回数
  Serial.println(String(cugo_button_press_time())); //ボタンの押されている時間
  Serial.println("距離センサ");    
  Serial.println(analogRead(PIN_SENSOR));
    if(cugo_check_button_times() > 2){
    cugo_reset_button_times(); //現状の押された回数  
    }
    if(analogRead(PIN_SENSOR) > 0.3){
    Serial.println("flag!!");
    cugo_stop(cugo_motor_controllers);
    }
  */
      Serial.println("自動走行モード終了"); 
      cugoRunMode = CUGO_RC_MODE; //自動走行モードをループしたい場合はCUGO_ARDUINO_MODEに変更
  }
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
      PIN_DOWN(1);//たおした瞬間にリセットになっているかを確認
      if (CUGO_ARDUINO_MODE_IN < time[1])
      { //KOPPROのBchを右に倒すとArduinoリセット
      resetFunc();
      } 
      if (CUGO_ARDUINO_MODE_OUT > time[1])
      { //KOPPROのBchを左に倒すとモードフラグの変更
            cugoRunMode = CUGO_ARDUINO_MODE;
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

  if(OLD_CMD_BUTTON_VALUE != digitalRead(CMD_BUTTON_PIN)){
    if(LOW == OLD_CMD_BUTTON_VALUE){ // 立ち上がり時の処理
      PIN_UP(3);
      cugo_button_check = false;//★ボタンはこれで判定でよいか？
    }
    else{ // 立下り時の処理
      PIN_DOWN(3);
      PIN_UP(3);
      cugo_button_check = true;
      if(CUGO_BUTTON_CHECK_BORDER < time[3] ){ 
          cugo_button_count++;
      }      
    }
    OLD_CMD_BUTTON_VALUE = OLD_CMD_BUTTON_VALUE ? LOW : HIGH;
  }  
}

//CUGOのセットアップ関連 
void cugo_setup()
{ //初期化実行
  cugo_init();
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
//エンコーダー割り込み時の実行処理 
void cugoLeftEncHandler(){
  cugo_motor_controllers[MOTOR_LEFT].updateEnc();
}
//エンコーダー割り込み時の実行処理
void cugoRightEncHandler(){
  cugo_motor_controllers[MOTOR_RIGHT].updateEnc();
}
