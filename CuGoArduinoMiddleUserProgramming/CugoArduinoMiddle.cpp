#include "CugoArduinoMiddle.h"
#include "Arduino.h"

/***** ↓必要に応じて各ユーザーごとに設定可能↓ *****/
// 回転方向ソフトウェア切り替え
const bool CUGO_L_reverse = false;
const bool CUGO_R_reverse = true;
// joshibi: L:True, R:false
// cugo-chan: L:false, R:True
/***** ↑必要に応じて各ユーザーごとに設定可能↑ *****/

//モード切替関連
  int cugoRunMode = CUGO_ARDUINO_MODE;
  int cugo_old_runmode = CUGO_ARDUINO_MODE;

//モータ制御関連
  bool cugo_direction_L; //true:forward false:backward
  bool cugo_direction_R; //true:forward false:backward
  long int cugo_target_count_L = 0;
  long int cugo_target_count_R = 0;
  
//ボタン・プロポ入力関連
  bool cugo_button_check = false;
  int cugo_button_count =0;  
  int CUGO_OLD_PWM_IN_PIN0_VALUE;   // プロポスティック入力値(L)
  int CUGO_OLD_PWM_IN_PIN1_VALUE;   // プロポスティック入力値(MODE)
  int CUGO_OLD_PWM_IN_PIN2_VALUE;   // プロポスティック入力値(R)
  int CUGO_OLD_CMD_BUTTON_VALUE = 0; 
  volatile unsigned long cugoUpTime[CUGO_PWM_IN_MAX];
  volatile unsigned long cugoRcTime[CUGO_PWM_IN_MAX];
  volatile unsigned long cugo_time[CUGO_PWM_IN_MAX];
  volatile unsigned long long cugoButtonTime;
//オドメトリ関連
  float cugo_odometer_theta = 0.0;
  float cugo_odometer_degree = 0.0;
  float cugo_odometer_x = 0.0;
  float cugo_odometer_y = 0.0;
  long int cugo_count_prev_L = 0;
  long int cugo_count_prev_R = 0;
  long int cugo_odometer_count_theta =0;
  unsigned long long int calc_odometer_time = 0;

//各種関数
//初期化関数
void cugo_init(){
  cugo_init_display();
  cugo_init_KOPROPO(CUGO_OLD_PWM_IN_PIN0_VALUE,CUGO_OLD_PWM_IN_PIN1_VALUE,CUGO_OLD_PWM_IN_PIN2_VALUE);
  cugo_reset_button_times();
  cugo_reset_odometer();
  pinMode(CUGO_PIN_ENCODER_L_A, INPUT_PULLUP);     //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  pinMode(CUGO_PIN_ENCODER_L_B, INPUT_PULLUP);     //B相用信号入力　内蔵プルアップ有効
  pinMode(CUGO_PIN_ENCODER_R_A, INPUT_PULLUP);     //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  pinMode(CUGO_PIN_ENCODER_R_B, INPUT_PULLUP);     //B相用信号入力　内蔵プルアップ有効    
}

//モード切り替わり確認
void cugo_check_mode_change(MotorController cugo_motor_controllers[CUGO_MOTOR_NUM])
{
  noInterrupts();      //割り込み停止
  cugoRcTime[0] = cugo_time[0];
  cugoRcTime[1] = cugo_time[1];
  cugoRcTime[2] = cugo_time[2];
  cugoButtonTime = cugo_time[3];
  interrupts();     //割り込み開始
  
  if ((cugoRunMode == CUGO_ARDUINO_MODE) && (cugo_old_runmode == CUGO_RC_MODE))
  {
    Serial.println(F("### MODE:CUGO_ARDUINO_MODE ###"));
    digitalWrite(LED_BUILTIN, HIGH);  // CUGO_ARDUINO_MODEでLED点灯           
    cugo_old_runmode = CUGO_ARDUINO_MODE;
    cugo_reset_pid_gain(cugo_motor_controllers);
    cugo_motor_direct_instructions(1500, 1500,cugo_motor_controllers); //直接停止命令を出す
    cugo_wait(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。    
  }
  if(cugoRunMode == CUGO_RC_MODE && cugo_old_runmode == CUGO_ARDUINO_MODE)
  {
    Serial.println(F("###   MODE:CUGO_RC_MODE    ###"));
    digitalWrite(LED_BUILTIN, LOW); // CUGO_RC_MODEでLED消灯
    cugo_old_runmode = CUGO_RC_MODE;            
    cugo_reset_pid_gain(cugo_motor_controllers);
    cugo_motor_direct_instructions(1500, 1500,cugo_motor_controllers); //直接停止命令を出す
    cugo_wait(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。    
  }                       
}

void cugo_wait(unsigned long long int  wait_ms){
  //約70分まで計測可能
  //例１時間待機したい場合：cugo_wait(120UL*60UL*1000UL);
unsigned long long int cugo_target_wait_time = wait_ms*1000ull;
unsigned long long int MAX_MICROS = 4294967295ull; // micros() 関数で計測する最大値
unsigned long long int startMicros = 0; // 計測開始時刻
unsigned long long int elapsedMicros = 0; // 経過時間（マイクロ秒単位）
unsigned long long int currentMicros = micros();

if(cugo_target_wait_time < MAX_MICROS){
  while(elapsedMicros < cugo_target_wait_time){
    
    if (startMicros == 0) {
      startMicros = micros();  // 計測開始時刻が初期化されていない場合、初期化する
    }
    currentMicros = micros();//時刻を取得
  
    // 経過時間を計算する
    if (currentMicros >= startMicros) {
      elapsedMicros = currentMicros - startMicros;
    } else {
      // オーバーフローが発生した場合
      elapsedMicros = (MAX_MICROS - startMicros) + currentMicros + 1;
    }
  }
}else{
  Serial.println("##WARNING::cugo_waitの計測可能時間を超えています。##");
}
  
}

void cugo_long_wait(unsigned long long int wait_seconds){
  //例 24時間計測したい場合：cugo_wait(24UL*60UL*60UL);

unsigned long long int  cugo_target_wait_time = wait_seconds*1000ull;
unsigned long long int MAX_MILLIS = 4294967295ull; // millis() 関数で計測する最大値
unsigned long long int startMillis = 0; // 計測開始時刻
unsigned long long int elapsedMillis = 0; // 経過時間（マイクロ秒単位）
unsigned long long int currentMillis = millis();

if(cugo_target_wait_time < MAX_MILLIS){
  
    while(elapsedMillis < cugo_target_wait_time){    
      if (startMillis == 0) {
        startMillis = millis();  // 計測開始時刻が初期化されていない場合、初期化する
      }
      currentMillis = millis();//現在時刻を取得
  
      // 経過時間を計算する
      if (currentMillis >= startMillis) {
        elapsedMillis = currentMillis - startMillis;
      } else {
        // オーバーフローが発生した場合
        elapsedMillis = (MAX_MILLIS - startMillis) + currentMillis + 1;
      }
    }
}else{
  Serial.println("##WARNING::cugo_long_waitの計測可能時間を超えています。##");
}

}

void cugo_move_pid(float target_rpm,bool use_pid,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  cugo_reset_pid_gain(cugo_motor_controllers);
  cugo_start_odometer();          

  //PID制御値
  float l_count_p =0 ;  
  float l_count_i =0 ;      
  float l_count_d =0 ;  
  float r_count_p =0 ;  
  float r_count_i =0 ;  
  float r_count_d =0 ;  
  // PID位置制御のデータ格納
  float l_count_prev_i_ =0 ;
  float l_count_prev_p_ =0 ;
  //float l_count_prev_p_ = (cugo_target_count_L - cugo_motor_controllers[CUGO_MOTOR_LEFT].getCount())/10000.0;
  float r_count_prev_i_ =0 ;
  float r_count_prev_p_ =0 ;
  //float r_count_prev_p_ = (cugo_target_count_R - cugo_motor_controllers[CUGO_MOTOR_RIGHT].getCount())/10000.0 ;
  float l_count_gain =0 ;
  float r_count_gain =0 ;

  if(target_rpm <= 0){
    Serial.println("##WARNING::目標速度が0以下のため進みません##");          
  }else if(cugo_target_count_L == 0 && cugo_target_count_R == 0){
    Serial.println("##WARNING::目標距離が左右ともに0のため進みません##");          
  }else{
    
    if(cugo_target_count_L >= 0){
      cugo_direction_L = true;
    }else{
      cugo_direction_L = false;
    }
    if(cugo_target_count_R >= 0){
      cugo_direction_R = true;
    }else{
      cugo_direction_R = false;
    }

    if(!use_pid){
      if(cugo_direction_L){
       cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(target_rpm);
      }else{
        cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(-target_rpm);
      }
      if(cugo_direction_R){
       cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(target_rpm);
      }else{
      cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(-target_rpm);
      }
    }

    if(abs(cugo_motor_controllers[CUGO_MOTOR_LEFT].getTargetRpm())>180 || abs(cugo_motor_controllers[CUGO_MOTOR_RIGHT].getTargetRpm()) > 180){
      Serial.println("##WARNING::目標速度が上限を超えているため正確な軌道を進まない可能性があります。##");          
    }

       
    //cugo_test時確認用
    Serial.println("start_count l:r:" + String(cugo_motor_controllers[CUGO_MOTOR_LEFT].getCount())+" ,"+ String(cugo_motor_controllers[CUGO_MOTOR_RIGHT].getCount()));  
    Serial.println("target_rpm l:r:" + String(cugo_motor_controllers[CUGO_MOTOR_LEFT].getTargetRpm())+" ,"+ String(cugo_motor_controllers[CUGO_MOTOR_RIGHT].getTargetRpm()));  
    Serial.println("target_count l:r:" + String(cugo_target_count_L)+" ,"+ String(cugo_target_count_R));    
    

    while(!cugo_check_count_achievement(CUGO_MOTOR_LEFT,cugo_motor_controllers) || !cugo_check_count_achievement(CUGO_MOTOR_RIGHT,cugo_motor_controllers)){  
      if(cugo_target_count_L == 0 && cugo_target_count_R == 0)
      {
        //停止しているだけの時
        cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(0);
        cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(0);
      } else{
        if(use_pid){
          // 各制御値の計算
          l_count_p = (cugo_target_count_L - cugo_motor_controllers[CUGO_MOTOR_LEFT].getCount())/10000.0;
          l_count_i = l_count_prev_i_ + l_count_p;
          l_count_d = l_count_p - l_count_prev_p_;
          r_count_p = (cugo_target_count_R - cugo_motor_controllers[CUGO_MOTOR_RIGHT].getCount())/10000.0;
          r_count_i = r_count_prev_i_ + r_count_p;
          r_count_d = r_count_p - r_count_prev_p_;

          l_count_i = min( max(l_count_i,-CUGO_L_MAX_COUNT_I),CUGO_L_MAX_COUNT_I);        
          r_count_i = min( max(r_count_i,-CUGO_R_MAX_COUNT_I),CUGO_R_MAX_COUNT_I);
          // PID制御
          l_count_gain = (l_count_p * CUGO_L_COUNT_KP + l_count_i * CUGO_L_COUNT_KI + l_count_d * CUGO_L_COUNT_KD);  
          r_count_gain = (r_count_p * CUGO_R_COUNT_KP + r_count_i * CUGO_R_COUNT_KI + r_count_d * CUGO_R_COUNT_KD);  
          // prev_ 更新
          l_count_prev_p_ = l_count_p;
          l_count_prev_i_ = l_count_i;
          r_count_prev_p_ = r_count_p;
          r_count_prev_i_ = r_count_i;
          l_count_gain = min( max(l_count_gain,-CUGO_MAX_MOTOR_RPM),CUGO_MAX_MOTOR_RPM);//モーターの速度上限        
          r_count_gain = min( max(r_count_gain,-CUGO_MAX_MOTOR_RPM),CUGO_MAX_MOTOR_RPM);//モーターの速度上限             
          l_count_gain = min( max(l_count_gain,-fabsf(target_rpm)),fabsf(target_rpm));//ユーザ設定の速度上限        
          r_count_gain = min( max(r_count_gain,-fabsf(target_rpm)),fabsf(target_rpm));//ユーザ設定の速度上限  
   
          //位置制御
          if(!cugo_check_count_achievement(CUGO_MOTOR_LEFT,cugo_motor_controllers)){
          cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(l_count_gain);          
          } 
          if(!cugo_check_count_achievement(CUGO_MOTOR_RIGHT,cugo_motor_controllers)){
          cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(r_count_gain);
          }

        }
      }
      for (int i = 0; i < CUGO_MOTOR_NUM; i++){ 
      cugo_motor_controllers[i].driveMotor();
      }
      cugo_wait(10);
      cugo_calc_odometer(cugo_motor_controllers);
    }
        
    //cugo_test時確認用
    Serial.println("result_odometer x,y,degree:" + String(cugo_check_odometer(CUGO_ODO_X))+" ,"+ String(cugo_check_odometer(CUGO_ODO_Y))+" ,"+ String(cugo_check_odometer(CUGO_ODO_THETA)));      
    Serial.println("result_count l:r:" + String(cugo_motor_controllers[CUGO_MOTOR_LEFT].getCount())+" ,"+ String(cugo_motor_controllers[CUGO_MOTOR_RIGHT].getCount()));
    Serial.println("===========");
    
  }
  cugo_stop(cugo_motor_controllers);   
  cugo_reset_pid_gain(cugo_motor_controllers);                 
}

  //前進制御＆回転制御
void cugo_move_forward(float target_distance,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  cugo_calc_necessary_count(target_distance,cugo_motor_controllers);
  cugo_move_pid(CUGO_NORMAL_MOTOR_RPM,true,cugo_motor_controllers);
  }
void cugo_move_forward(float target_distance,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  cugo_calc_necessary_count(target_distance,cugo_motor_controllers);
  cugo_move_pid(target_rpm,true,cugo_motor_controllers);
  }
void cugo_move_forward_raw(float target_distance,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  cugo_calc_necessary_count(target_distance,cugo_motor_controllers);
  cugo_move_pid(target_rpm,false,cugo_motor_controllers);
  }
void cugo_turn_clockwise(float target_degree,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  cugo_calc_necessary_rotate(target_degree,cugo_motor_controllers);  
  cugo_move_pid(CUGO_NORMAL_MOTOR_RPM,true,cugo_motor_controllers);  
  }
void cugo_turn_clockwise(float target_degree,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  cugo_calc_necessary_rotate(target_degree,cugo_motor_controllers);
  cugo_move_pid(target_rpm,true,cugo_motor_controllers);
  }
void cugo_turn_clockwise_raw(float target_degree,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  cugo_calc_necessary_rotate(target_degree,cugo_motor_controllers);
  cugo_move_pid(target_rpm,false,cugo_motor_controllers);
  }
void cugo_turn_counterclockwise(float target_degree,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  cugo_calc_necessary_rotate(-target_degree,cugo_motor_controllers);
  cugo_move_pid(CUGO_NORMAL_MOTOR_RPM,true,cugo_motor_controllers);    
  }
void cugo_turn_counterclockwise(float target_degree,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  cugo_calc_necessary_rotate(-target_degree,cugo_motor_controllers);
  cugo_move_pid(target_rpm,true,cugo_motor_controllers);  
  }
void cugo_turn_counterclockwise_raw(float target_degree,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  cugo_calc_necessary_rotate(-target_degree,cugo_motor_controllers);
  cugo_move_pid(target_rpm,false,cugo_motor_controllers);  
  }
  //円軌道での移動命令
void cugo_curve_theta_raw(float target_radius,float target_theta,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  cugo_reset_pid_gain(cugo_motor_controllers);
  cugo_start_odometer();          
  cugo_target_count_L = (target_radius-CUGO_TREAD/2)*(target_theta*PI/180)*CUGO_CONVERSION_DISTANCE_TO_COUNT;
  cugo_target_count_R = (target_radius+CUGO_TREAD/2)*(target_theta*PI/180)*CUGO_CONVERSION_DISTANCE_TO_COUNT;      



  if(target_rpm <= 0){
    Serial.println("##WARNING::目標速度が0以下のため進みません##");          
  }else if(cugo_target_count_L == 0 && cugo_target_count_R == 0){
    Serial.println("##WARNING::目標距離が左右ともに0のため進みません##");          
  }else{
    if(target_radius<0){
      cugo_target_count_L = -cugo_target_count_L;
      cugo_target_count_R = -cugo_target_count_R; 
    }
    if(target_theta>0){
      if(target_radius != 0){
        cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(target_rpm*((target_radius-CUGO_TREAD/2)/target_radius));
        cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(target_rpm*((target_radius+CUGO_TREAD/2)/target_radius));
      }else{
        cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(-target_rpm);
        cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(target_rpm);      
      }
    }else if(target_theta<0){
      if(target_radius != 0){
        cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(-target_rpm*((target_radius-CUGO_TREAD/2)/target_radius));
        cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(-target_rpm*((target_radius+CUGO_TREAD/2)/target_radius));
      }else{
        cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(target_rpm);
        cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(-target_rpm);      
      }
    }else if(target_theta=0){
      cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(0);
      cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(0);     
    }else{
    
    }

    if(abs(cugo_motor_controllers[CUGO_MOTOR_LEFT].getTargetRpm())>CUGO_MAX_MOTOR_RPM || abs(cugo_motor_controllers[CUGO_MOTOR_RIGHT].getTargetRpm()) > CUGO_MAX_MOTOR_RPM){
      Serial.println("##WARNING::目標速度が上限を超えているため正確な軌道を進まない可能性があります。##");          
    }

    if(abs(cugo_motor_controllers[CUGO_MOTOR_LEFT].getTargetRpm()) < 20 || abs(cugo_motor_controllers[CUGO_MOTOR_RIGHT].getTargetRpm()) < 20){
      Serial.println("##WARNING::目標速度が十分な速度ではないため正確な軌道を進まない可能性があります。##");          
    }

    if(cugo_target_count_L >= 0){
      cugo_direction_L = true;
    }else{
      cugo_direction_L = false;
    }

    if(cugo_target_count_R >= 0){
      cugo_direction_R = true;
    }else{
      cugo_direction_R = false;
    }
    
     
    //cugo_test時確認用
    Serial.println("start_count l:r:" + String(cugo_motor_controllers[CUGO_MOTOR_LEFT].getCount())+" ,"+ String(cugo_motor_controllers[CUGO_MOTOR_RIGHT].getCount()));  
    Serial.println("target_rpm l:r:" + String(cugo_motor_controllers[CUGO_MOTOR_LEFT].getTargetRpm())+" ,"+ String(cugo_motor_controllers[CUGO_MOTOR_RIGHT].getTargetRpm()));  
    Serial.println("target_count l:r:" + String(cugo_target_count_L)+" ,"+ String(cugo_target_count_R));    
    
    while(!cugo_check_count_achievement(CUGO_MOTOR_LEFT,cugo_motor_controllers) || !cugo_check_count_achievement(CUGO_MOTOR_RIGHT,cugo_motor_controllers)){  
      if(cugo_target_count_L == 0 && cugo_target_count_R == 0)
      {
        //停止しているだけの時
        cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(0);
        cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(0);
      }
      for (int i = 0; i < CUGO_MOTOR_NUM; i++){ 
      cugo_motor_controllers[i].driveMotor();
      }
      cugo_wait(10);
      cugo_calc_odometer(cugo_motor_controllers);
    }
       
    //cugo_test時確認用
    Serial.println("result_odometer x,y,degree:" + String(cugo_check_odometer(CUGO_ODO_X))+" ,"+ String(cugo_check_odometer(CUGO_ODO_Y))+" ,"+ String(cugo_check_odometer(CUGO_ODO_THETA)));      
    Serial.println("result_count l:r:" + String(cugo_motor_controllers[CUGO_MOTOR_LEFT].getCount())+" ,"+ String(cugo_motor_controllers[CUGO_MOTOR_RIGHT].getCount()));
    Serial.println("===========");
    
  }
  cugo_stop(cugo_motor_controllers); 
  cugo_reset_pid_gain(cugo_motor_controllers);          
  }
void cugo_curve_distance_raw(float target_radius,float target_distance,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  cugo_reset_pid_gain(cugo_motor_controllers);
  cugo_start_odometer();
  if(target_radius == 0){          
    Serial.println("##WARNING::軌道半径が0のため進みません##");
  }else{
    cugo_target_count_L = target_distance*((target_radius-CUGO_TREAD/2)/target_radius)*CUGO_CONVERSION_DISTANCE_TO_COUNT;
    cugo_target_count_R = target_distance*((target_radius+CUGO_TREAD/2)/target_radius)*CUGO_CONVERSION_DISTANCE_TO_COUNT;
        
    if(target_rpm <= 0){
      Serial.println("##WARNING::目標速度が0以下のため進みません##");          
    }else if(cugo_target_count_L == 0 && cugo_target_count_R == 0){
    Serial.println("##WARNING::目標距離が左右ともに0のため進みません##");          
    }else{
    
      if(target_distance>0){
        if(target_radius != 0){
          cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(target_rpm*((target_radius-CUGO_TREAD/2)/target_radius));
          cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(target_rpm*((target_radius+CUGO_TREAD/2)/target_radius));
        }else{
          cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(-target_rpm);
          cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(target_rpm);      
        }
      }else if(target_distance<0){
        if(target_radius != 0){
          cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(-target_rpm*((target_radius-CUGO_TREAD/2)/target_radius));
          cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(-target_rpm*((target_radius+CUGO_TREAD/2)/target_radius));
        }else{
          cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(target_rpm);
          cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(-target_rpm);      
        }
      }else if(target_distance=0){
        cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(0);
        cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(0);     
      }else{
    
      }

      if(abs(cugo_motor_controllers[CUGO_MOTOR_LEFT].getTargetRpm())>CUGO_MAX_MOTOR_RPM || abs(cugo_motor_controllers[CUGO_MOTOR_RIGHT].getTargetRpm()) > CUGO_MAX_MOTOR_RPM){
        Serial.println("##WARNING::目標速度が上限を超えているため正確な軌道を進まない可能性があります。##");          
      }

      if(abs(cugo_motor_controllers[CUGO_MOTOR_LEFT].getTargetRpm()) < 20 || abs(cugo_motor_controllers[CUGO_MOTOR_RIGHT].getTargetRpm()) < 20){
        Serial.println("##WARNING::目標速度が十分な速度ではないため正確な軌道を進まない可能性があります。##");          
      }

      if(cugo_target_count_L >= 0){
        cugo_direction_L = true;
      }else{
        cugo_direction_L = false;
      }

      if(cugo_target_count_R >= 0){
        cugo_direction_R = true;
      }else{
        cugo_direction_R = false;
      }
         
      //cugo_test時確認用
      Serial.println("start_count l:r:" + String(cugo_motor_controllers[CUGO_MOTOR_LEFT].getCount())+" ,"+ String(cugo_motor_controllers[CUGO_MOTOR_RIGHT].getCount()));  
      Serial.println("target_rpm l:r:" + String(cugo_motor_controllers[CUGO_MOTOR_LEFT].getTargetRpm())+" ,"+ String(cugo_motor_controllers[CUGO_MOTOR_RIGHT].getTargetRpm()));  
      Serial.println("target_count l:r:" + String(cugo_target_count_L)+" ,"+ String(cugo_target_count_R));
         
      while(!cugo_check_count_achievement(CUGO_MOTOR_LEFT,cugo_motor_controllers) || !cugo_check_count_achievement(CUGO_MOTOR_RIGHT,cugo_motor_controllers)){  
        if(cugo_target_count_L == 0 && cugo_target_count_R == 0)
        {
          //停止しているだけの時
          cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(0);
          cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(0);
        }
        for (int i = 0; i < CUGO_MOTOR_NUM; i++){ 
          cugo_motor_controllers[i].driveMotor();
        }
        cugo_wait(10);     
        cugo_calc_odometer(cugo_motor_controllers);    
      }
        
      //cugo_test時確認用      
      Serial.println("result_odometer x,y,degree:" + String(cugo_check_odometer(CUGO_ODO_X))+" ,"+ String(cugo_check_odometer(CUGO_ODO_Y))+" ,"+ String(cugo_check_odometer(CUGO_ODO_THETA)));      
      Serial.println("result_count l:r:" + String(cugo_motor_controllers[CUGO_MOTOR_LEFT].getCount())+" ,"+ String(cugo_motor_controllers[CUGO_MOTOR_RIGHT].getCount()));
      Serial.println("===========");
      
    }
  }
  cugo_stop(cugo_motor_controllers); 
  cugo_reset_pid_gain(cugo_motor_controllers);
  }
  //チェック関連
bool cugo_check_count_achievement(int motor_num_,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
    long int target_count_ = 0;
    bool cugo_direction_;//前進後進方向の変数変更　
    if(motor_num_ == CUGO_MOTOR_LEFT){
      target_count_ = cugo_target_count_L;
      cugo_direction_ = cugo_direction_L;
    }else if(motor_num_ == CUGO_MOTOR_RIGHT){
      target_count_ = cugo_target_count_R;
      cugo_direction_ = cugo_direction_R;      
    }else{
    return false;
    }
    
    // 目標達成チェック
    if(cugo_direction_){
      if (target_count_<=cugo_motor_controllers[motor_num_].getCount()){
        cugo_motor_controllers[motor_num_].setTargetRpm(0);
        return true;
      } 
    }else{
      if (target_count_>=cugo_motor_controllers[motor_num_].getCount()){
        cugo_motor_controllers[motor_num_].setTargetRpm(0);
        return true;
      } 
    }

    return false;
  }
int cugo_check_propo_channel_value(int channel_number){
  noInterrupts();      //割り込み停止
  cugoRcTime[0] = cugo_time[0];
  cugoRcTime[1] = cugo_time[1];
  cugoRcTime[2] = cugo_time[2];  
  interrupts();     //割り込み開始  

  if(channel_number == CUGO_PROPO_A){
  return cugoRcTime[0];    
  }else if(channel_number == CUGO_PROPO_B){
  return cugoRcTime[1];    
  }else if(channel_number == CUGO_PROPO_C){
  return cugoRcTime[2];  
  }else{
  return 0;    
  }
  return 0;

  }
bool cugo_check_button(){
  return cugo_button_check;  
  }
int cugo_check_button_times(){
  
  return cugo_button_count;  
  }
void cugo_reset_button_times(){
  cugo_button_count = 0;
  }
long int cugo_button_press_time(){
  //割り込み停止
  if(cugo_button_check){
  noInterrupts();
    CUGO_PIN_DOWN(3);
    cugoButtonTime = cugo_time[3];
  interrupts();     
  //割り込み開始
    return cugoButtonTime/1000;  
  }else{
    return 0;      
  }


  }
float cugo_check_odometer(int check_number){
  //odometer_number 0:x,1:y,2:theta
  if(check_number == CUGO_ODO_X){
  return cugo_odometer_x;    
  }else if(check_number == CUGO_ODO_Y){
  return cugo_odometer_y;    
  }else if(check_number == CUGO_ODO_THETA){
  return cugo_odometer_theta;
  }else if(check_number == CUGO_ODO_DEGREE){
  return cugo_odometer_degree;  
  }else{
  return 0;    
  }
  return 0;

  }
void cugo_calc_odometer(MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  if(calc_odometer_time < 10000 + micros()){
    long int cugo_dif_count_theta_ =(cugo_motor_controllers[CUGO_MOTOR_RIGHT].getCount()-cugo_count_prev_R)-(cugo_motor_controllers[CUGO_MOTOR_LEFT].getCount()-cugo_count_prev_L);
    long int cugo_dif_count_v_ =((cugo_motor_controllers[CUGO_MOTOR_RIGHT].getCount()-cugo_count_prev_R)+(cugo_motor_controllers[CUGO_MOTOR_LEFT].getCount()-cugo_count_prev_L))/2;
    cugo_odometer_theta += cugo_dif_count_theta_*(CUGO_CONVERSION_COUNT_TO_DISTANCE/CUGO_TREAD);

    cugo_odometer_x += (cugo_dif_count_v_ * cos(cugo_odometer_theta) )*CUGO_CONVERSION_COUNT_TO_DISTANCE;
    cugo_odometer_y += (cugo_dif_count_v_ * sin(cugo_odometer_theta) )*CUGO_CONVERSION_COUNT_TO_DISTANCE;    
    cugo_odometer_degree = cugo_odometer_theta*180/PI;
    cugo_count_prev_L = cugo_motor_controllers[CUGO_MOTOR_LEFT].getCount();
    cugo_count_prev_R = cugo_motor_controllers[CUGO_MOTOR_RIGHT].getCount();  
    calc_odometer_time = micros();
  }
  }
void cugo_reset_odometer(){
  cugo_count_prev_L = 0;
  cugo_count_prev_R = 0;
  cugo_odometer_count_theta =0;
  cugo_odometer_x = 0 ;
  cugo_odometer_y = 0 ;
  cugo_odometer_theta = 0 ;
  cugo_odometer_degree = 0 ;
  calc_odometer_time = micros();
  }
void cugo_start_odometer(){
  cugo_count_prev_L = 0;
  cugo_count_prev_R = 0;
  calc_odometer_time = micros();
  }
void cugo_init_KOPROPO(int CUGO_OLD_PWM_IN_PIN0_VALUE,int CUGO_OLD_PWM_IN_PIN1_VALUE,int CUGO_OLD_PWM_IN_PIN2_VALUE){
  //Serial.println(F("#   cugo_init_KOPROPO"));//確認用
  // ピン変化割り込みの初期状態保存
  cugoRunMode = CUGO_RC_MODE;
  CUGO_OLD_PWM_IN_PIN0_VALUE = digitalRead(CUGO_PWM_IN_PIN0);
  CUGO_OLD_PWM_IN_PIN1_VALUE = digitalRead(CUGO_PWM_IN_PIN1);
  CUGO_OLD_PWM_IN_PIN2_VALUE = digitalRead(CUGO_PWM_IN_PIN2);
  CUGO_OLD_PWM_IN_PIN2_VALUE = digitalRead(CUGO_CMD_BUTTON_PIN);
  
  // ピン変化割り込みの設定（D5,D6,D7をレジスタ直接読み取りで割り込み処理）
  pinMode(CUGO_PWM_IN_PIN0, INPUT);
  pinMode(CUGO_PWM_IN_PIN1, INPUT);
  pinMode(CUGO_PWM_IN_PIN2, INPUT);
  pinMode(CUGO_CMD_BUTTON_PIN, INPUT_PULLUP);
  
  //PCMSK1 |= B00000100;  // A2を有効 :PCINT10
  //PCMSK1 |= (1 << INT10);
  //PCICR  |= (1 << PCIE1);
  //PCMSK2 |= (1 << INT5);
  //PCMSK2 |= (1 << INT6);
  //PCMSK2 |= (1 << INT7);
  //PCICR  |= (1 << PCIE2);
  PCMSK2 |= B11100000;  // D5,6,7を有効 :PCINT21,22,23
  PCICR  |= B00000110;  // PCIE1,2を有効

  pinMode(LED_BUILTIN, OUTPUT); // Arduino/RC MODEの表示
       cugo_wait(100);
  }
void cugo_calc_necessary_rotate(float degree,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]) {
  //Serial.println(F("#   cugo_calc_necessary_rotate"));//確認用
  cugo_target_count_L = ((degree / 360) * CUGO_TREAD * PI) * CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_L * PI);
  cugo_target_count_R = -((degree / 360) * CUGO_TREAD * PI) * CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_R * PI);
  //Serial.println("degree: " + String(degree));
  //Serial.println("### cugo_target_count_L/R: " + String(*cugo_target_count_L) + " / " + String(*cugo_target_count_R) + "###");
  //Serial.println("kakudo: " + String((degree / 360) * CUGO_TREAD * PI));
  //Serial.println("PI: " + String(PI));
  //Serial.println("issyuu: " + String(2 * CUGO_WHEEL_RADIUS_R * PI));
  }
void cugo_calc_necessary_count(float distance,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]) {
  //Serial.println(F("#   cugo_calc_necessary_count"));//確認用

  //cugo_target_count_L = distance / (2 * CUGO_WHEEL_RADIUS_L * PI);
  //cugo_target_count_R = distance / (2 * CUGO_WHEEL_RADIUS_R * PI);
  //cugo_target_count_L = cugo_target_count_L * CUGO_ENCODER_RESOLUTION;
  //cugo_target_count_R = cugo_target_count_R * CUGO_ENCODER_RESOLUTION;
  cugo_target_count_L = distance * CUGO_CONVERSION_DISTANCE_TO_COUNT;
  cugo_target_count_R = distance * CUGO_CONVERSION_DISTANCE_TO_COUNT;
  //Serial.println("distance: " + String(distance));
  //Serial.println("distance: " + String(CUGO_ENCODER_RESOLUTION));
  //Serial.println("2 * CUGO_WHEEL_RADIUS_L * PI: " + String(2 * CUGO_WHEEL_RADIUS_L * PI));
  //Serial.println("calc: " + String(distance * CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_L * PI)));
  //Serial.println("### cugo_target_count_L/R: " + String(*cugo_target_count_L) + " / " + String(*cugo_target_count_R) + "###");
  //Serial.println("distance: " + String(distance));
  //Serial.println("CUGO_WHEEL_RADIUS_L: " + String(CUGO_WHEEL_RADIUS_L));
  //Serial.println("PI: " + String(PI));
  //Serial.println("issyuu: " + String(2 * CUGO_WHEEL_RADIUS_R * PI));

  }
void cugo_motor_direct_instructions(int left, int right,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  //Serial.println(F("#   cugo_motor_direct_instructions"));//確認用
  cugo_motor_controllers[CUGO_MOTOR_LEFT].servo_.writeMicroseconds(left);
  cugo_motor_controllers[CUGO_MOTOR_RIGHT].servo_.writeMicroseconds(right);
  }
void cugo_rcmode(volatile unsigned long cugoRcTime[CUGO_PWM_IN_MAX],MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  //Serial.println(F("#   cugo_CUGO_RC_MODE"));//確認用  
  digitalWrite(LED_BUILTIN, LOW); // CUGO_RC_MODEでLED消灯
  // 値をそのままへESCへ出力する
  if((cugoRcTime[0] < CUGO_PROPO_MAX && cugoRcTime[0] > CUGO_PROPO_MIN) && (cugoRcTime[2] < CUGO_PROPO_MAX && cugoRcTime[2] > CUGO_PROPO_MIN) ){
    cugo_motor_direct_instructions(cugoRcTime[0], cugoRcTime[2],cugo_motor_controllers);
  }

  }
void cugo_stop(MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  //Serial.println(F("#   cugo_stop"));//確認用
  cugo_motor_controllers[CUGO_MOTOR_LEFT].setTargetRpm(0.0);
  cugo_motor_controllers[CUGO_MOTOR_RIGHT].setTargetRpm(0.0);
  cugo_motor_direct_instructions(1500, 1500,cugo_motor_controllers);
  cugo_wait(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。    
  
  }
void cugo_reset_pid_gain(MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){
  //Serial.println(F("#   cugo_reset_pid_gain"));//確認用  
  for (int i = 0; i < CUGO_MOTOR_NUM; i++)
  {
    cugo_motor_controllers[i].reset_PID_param();
  }
  }
void cugo_init_display(){
  //delay(30);
  cugo_wait(30);

  /*  
  Serial.println("");
  Serial.println("");  
  Serial.println("#######################################");
  Serial.println("#######################################");
  Serial.println("#                                     #");
  Serial.println("#   ####    ##  ##    ####     ####   #");
  Serial.println("#  ##  ##   ##  ##   ##  ##   ##  ##  #");
  Serial.println("#  ##       ##  ##   ##       ##  ##  #");
  Serial.println("#  ##       ##  ##   ## ###   ##  ##  #");
  Serial.println("#  ##       ##  ##   ##  ##   ##  ##  #");
  Serial.println("#  ##  ##   ##  ##   ##  ##   ##  ##  #");
  Serial.println("#   ####     ####     ####     ####   #");
  Serial.println("#                                     #");
  Serial.println("#######################################");
  Serial.println("#######################################");
  Serial.println("");
  Serial.println("");  
  */
  Serial.println("###############################");  
  Serial.println("###   CugoAruduinoKitStart  ###");
  Serial.println("###############################");  

  }
void cugo_test(int test_number,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]){

  if(test_number == 0){//試験プログラムパターン⓪：サンプルプログラムテスト
    Serial.println("自動走行モード開始");  
    Serial.println("1.0mの正方形移動の実施");

    cugo_move_forward(1.0,cugo_motor_controllers);
    cugo_wait(1000);
    cugo_turn_clockwise(90,0,cugo_motor_controllers);
    cugo_wait(1000);
    cugo_move_forward(1.0,cugo_motor_controllers);
    cugo_wait(1000);
    cugo_turn_clockwise(90,0,cugo_motor_controllers);
    cugo_wait(1000);
    cugo_move_forward(1.0,cugo_motor_controllers);
    cugo_wait(1000);
    cugo_turn_clockwise(90,0,cugo_motor_controllers);
    cugo_wait(1000);
    cugo_move_forward(1.0,cugo_motor_controllers);
    cugo_wait(1000);
    cugo_turn_clockwise(90,0,cugo_motor_controllers);
    cugo_wait(1000);

    Serial.println("自動走行モード終了"); 
    cugoRunMode = CUGO_RC_MODE;  
  }
  if(test_number == 1){//試験プログラムパターン①：走行関連テスト
    Serial.println(F("自動走行モード開始"));
    unsigned long int cugo_test_start = micros();  
    Serial.println(F("cugo_move_forward(0.5,cugo_motor_controllers)"));
      cugo_move_forward(0.5,cugo_motor_controllers);
    Serial.println(F("cugo_move_forward(-0.5,cugo_motor_controllers)"));
      cugo_move_forward(-0.5,cugo_motor_controllers);      
    Serial.println(F("cugo_move_forward(0,cugo_motor_controllers)"));
      cugo_move_forward(0,cugo_motor_controllers); 
    
    Serial.println(F("cugo_move_forward_raw(0.5,70,cugo_motor_controllers)"));
      cugo_move_forward_raw(0.5,70,cugo_motor_controllers);      
    Serial.println(F("cugo_move_forward_raw(-0.5,40,cugo_motor_controllers)"));
      cugo_move_forward_raw(-0.5,40,cugo_motor_controllers);      

    Serial.println(F("cugo_turn_clockwise(90,cugo_motor_controllers)"));
      cugo_turn_clockwise(90,cugo_motor_controllers);
    Serial.println(F("cugo_turn_clockwise(-90,cugo_motor_controllers)"));
      cugo_turn_clockwise(-90,cugo_motor_controllers);
    Serial.println(F("cugo_turn_clockwise(0,cugo_motor_controllers)"));
      cugo_turn_clockwise(0,cugo_motor_controllers); 
    Serial.println(F("cugo_turn_clockwise(90,90,cugo_motor_controllers)"));
      cugo_turn_clockwise(90,90,cugo_motor_controllers);
    Serial.println(F("cugo_turn_clockwise(90,0,cugo_motor_controllers)"));
      cugo_turn_clockwise(90,0,cugo_motor_controllers);
    Serial.println(F("cugo_turn_clockwise(90,-90,cugo_motor_controllers)"));
      cugo_turn_clockwise(90,-90,cugo_motor_controllers); 

    Serial.println(F("cugo_turn_clockwise_raw(60,80,cugo_motor_controllers)"));
      cugo_turn_clockwise_raw(60,80,cugo_motor_controllers);
    Serial.println(F("cugo_turn_clockwise_raw(-60,40,cugo_motor_controllers)"));
      cugo_turn_clockwise_raw(-60,40,cugo_motor_controllers);

    Serial.println(F("cugo_turn_counterclockwise(90,cugo_motor_controllers)"));
      cugo_turn_counterclockwise(90,cugo_motor_controllers);
    Serial.println(F("cugo_turn_counterclockwise(-90,cugo_motor_controllers)"));
      cugo_turn_counterclockwise(-90,cugo_motor_controllers);
    Serial.println(F("cugo_turn_counterclockwise(0,cugo_motor_controllers)"));
      cugo_turn_counterclockwise(0,cugo_motor_controllers);
    Serial.println(F("cugo_turn_counterclockwise(90,90,cugo_motor_controllers)"));
      cugo_turn_counterclockwise(90,90,cugo_motor_controllers);
    Serial.println(F("cugo_turn_counterclockwise(90,0,cugo_motor_controllers)"));
      cugo_turn_counterclockwise(90,0,cugo_motor_controllers);
    Serial.println(F("cugo_turn_counterclockwise(90,-90,cugo_motor_controllers)"));
      cugo_turn_counterclockwise(90,-90,cugo_motor_controllers);

    Serial.println(F("cugo_turn_counterclockwise_raw(60,80,cugo_motor_controllers)"));
      cugo_turn_counterclockwise_raw(60,80,cugo_motor_controllers);
    Serial.println(F("cugo_turn_counterclockwise_raw(-60,40,cugo_motor_controllers)"));
      cugo_turn_counterclockwise_raw(-60,40,cugo_motor_controllers);

    Serial.println(F("cugo_curve_theta_raw(1.0,90,90,cugo_motor_controllers)"));
      cugo_curve_theta_raw(1.0,90,90,cugo_motor_controllers);
    Serial.println(F("cugo_curve_theta_raw(-1.0,90,90,cugo_motor_controllers)"));
      cugo_curve_theta_raw(-1.0,90,90,cugo_motor_controllers);
    Serial.println(F("cugo_curve_theta_raw(0,90,90,cugo_motor_controllers)"));
      cugo_curve_theta_raw(0,90,90,cugo_motor_controllers);
    Serial.println(F("cugo_curve_theta_raw(0.5,540,90,cugo_motor_controllers)"));
      cugo_curve_theta_raw(0.5,540,90,cugo_motor_controllers);
    Serial.println(F("cugo_curve_theta_raw(1.0,-90,90,cugo_motor_controllers)"));
      cugo_curve_theta_raw(1.0,-90,90,cugo_motor_controllers);
    Serial.println(F("cugo_curve_theta_raw(1.0,0,90,cugo_motor_controllers)"));
      cugo_curve_theta_raw(1.0,0,90,cugo_motor_controllers);
    Serial.println(F("cugo_curve_theta_raw(1.0,90,180,cugo_motor_controllers)"));
      cugo_curve_theta_raw(1.0,90,180,cugo_motor_controllers);
    Serial.println(F("cugo_curve_theta_raw(1.0,90,-90,cugo_motor_controllers)"));
      cugo_curve_theta_raw(1.0,90,-90,cugo_motor_controllers);
    Serial.println(F("cugo_curve_theta_raw(1.0,90,0,cugo_motor_controllers)"));
      cugo_curve_theta_raw(1.0,90,0,cugo_motor_controllers);
    Serial.println(F("cugo_curve_distance_raw(1.0,1.0,90,cugo_motor_controllers)"));
      cugo_curve_distance_raw(1.0,1.0,90,cugo_motor_controllers);
    Serial.println(F("cugo_curve_distance_raw(-1.0,1.0,90,cugo_motor_controllers)"));
      cugo_curve_distance_raw(-1.0,1.0,90,cugo_motor_controllers);    
    Serial.println(F("cugo_curve_distance_raw(0,1.0,90,cugo_motor_controllers)"));
      cugo_curve_distance_raw(0,1.0,90,cugo_motor_controllers);
    Serial.println(F("cugo_curve_distance_raw(0.5,12.0,90,cugo_motor_controllers)"));
      cugo_curve_distance_raw(0.5,12.0,90,cugo_motor_controllers);
    Serial.println(F("cugo_curve_distance_raw(1.0,-1.0,90,cugo_motor_controllers)"));
      cugo_curve_distance_raw(1.0,-1.0,90,cugo_motor_controllers);    
    Serial.println(F("cugo_curve_distance_raw(1.0,0,90,cugo_motor_controllers)"));
      cugo_curve_distance_raw(1.0,0,90,cugo_motor_controllers);
    Serial.println(F("cugo_curve_distance_raw(1.0,1.0,180,cugo_motor_controllers)"));
      cugo_curve_distance_raw(1.0,1.0,180,cugo_motor_controllers);
    Serial.println(F("cugo_curve_distance_raw(1.0,1.0,-90,cugo_motor_controllers)"));
      cugo_curve_distance_raw(1.0,1.0,-90,cugo_motor_controllers);    
    Serial.println(F("cugo_curve_distance_raw(1.0,1.0,0,cugo_motor_controllers)"));
      cugo_curve_distance_raw(1.0,1.0,0,cugo_motor_controllers);

    Serial.println(F("自動走行モード終了")); 
    Serial.println("処理時間(micros)::" + String(micros()-cugo_test_start)); 
    cugoRunMode = CUGO_RC_MODE;
  }
   
  if(test_number == 2){//試験プログラムパターン②：プロポ入力、ボタン関連テスト
  
  unsigned long int cugo_test_start = micros();  
  //試験用関数記載
  Serial.print("Ach::" + String(cugo_check_propo_channel_value(0))+"  Bch::" + String(cugo_check_propo_channel_value(1))+ "  Cch::" + String(cugo_check_propo_channel_value(2))); 
  Serial.print(", cugo_button_press_time::"+String(cugo_button_press_time()) +", cugo_check_button_times::"+ String(cugo_check_button_times()) +", cugo_check_button::"+ String(cugo_check_button()));
  cugo_wait(100);
  if(cugo_check_button_times() >10){
    Serial.println("");
    Serial.println("####################################");
    Serial.println("cugo_reset_button_times::count_reset");
    Serial.println("####################################");
    Serial.println("");
    cugo_reset_button_times();
  }  
  
  Serial.println(", 処理時間(micros)::" + String(micros()-cugo_test_start)); 
  cugo_wait(50);
  cugoRunMode = CUGO_ARDUINO_MODE; //自動走行モードをループしたい場合はCUGO_ARDUINO_MODEに変更
   
  }
  if(test_number == 3){//試験プログラムパターン③
  Serial.println("自動走行モード開始");  
  unsigned long int cugo_test_start = micros();  
  //試験用関数記載
    Serial.println("半径1.0mのS字移動");

    cugo_curve_distance_raw(1.0,180,90,cugo_motor_controllers);
    cugo_wait(1000);
    cugo_curve_distance_raw(-1.0,180,90,cugo_motor_controllers);
    cugo_wait(1000);

  Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
  Serial.println("自動走行モード終了"); 
  }
  
  if(test_number == 4){//試験プログラムパターン④ランダム試験
  Serial.println("自動走行モード開始");  
  unsigned long int cugo_test_start = micros();  
  //試験用関数記載

  /*
    //試験用関数記載
    unsigned long int  target_time_test;//32767
    target_time_test = 60*60*1000;
    Serial.println("test_time"+String(target_time_test));
    cugo_long_wait(target_time_test);
    Serial.println(String(target_time_test));
    Serial.println("done!");
    cugo_turn_clockwise(90,-20,cugo_motor_controllers);
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println("自動走行モード終了"); 

    Serial.println("自動走行モード開始");  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_turn_clockwise(90,0,cugo_motor_controllers);
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println("自動走行モード終了"); 

    Serial.println("自動走行モード開始");  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_turn_counterclockwise(90,-20,cugo_motor_controllers);
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println("自動走行モード終了"); 

    Serial.println("自動走行モード開始");  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_turn_counterclockwise(90,-20,cugo_motor_controllers);
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println("自動走行モード終了"); 

    Serial.println("自動走行モード開始");  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_curve_theta_raw(1.0,30,-90,cugo_motor_controllers);
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println("自動走行モード終了"); 

    Serial.println("自動走行モード開始");  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_curve_theta_raw(1.0,30,0,cugo_motor_controllers);
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
    Serial.println("自動走行モード終了"); 
    Serial.println("自動走行モード開始");  
    cugo_test_start = micros();  
    //試験用関数記載
    cugo_curve_distance_raw(0,18,90,cugo_motor_controllers);
    Serial.println("処理時間(micros)" + String(micros()-cugo_test_start)); 
  */
  Serial.println("自動走行モード終了"); 

  }

  }
