/****************************************************************************
 * CugoSDKSamples	                                            *
 *                                                                          *
 * Copyright [2023] [CuboRex.Inc]                                           *
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
/*****************CugoPicoSDKSamples ver1.00******************************/
/****************************************************************************
 *  CugoDKSamplesをご利用される方へ
 *  詳細はクイックリファレンス.mdをご確認ください。
 ****************************************************************************/

#include "CugoSDK.h"

void setup() {
  cugo_switching_reset = true;
  //プロポでラジコンモード切替時に初期化したい場合はtrue、初期化しない場合はfalse 

  cugo_init();//初期設定
}

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

    ld2_set_control_mode(CUGO_RC_MODE); 
    cugo_wait(1000);
    //自動走行モードを1回のloopで終了する場合のみ記載、不要の場合コメントアウト
  
  }



}
