#include "CugoArduinoMiddle.h"
#include "Arduino.h"

/***** ↓必要に応じて各ユーザーごとに設定可能↓ *****/
// 回転方向ソフトウェア切り替え
const bool L_reverse = false;
const bool R_reverse = true;
// joshibi: L:True, R:false
// cugo-chan: L:false, R:True
/***** ↑必要に応じて各ユーザーごとに設定可能↑ *****/
int cugo_old_runmode = CUGO_ARDUINO_MODE;
int cugo_button_count =0;

float cugo_odometer = 0.0;
long int cugo_count_prev_L = 0;
long int cugo_count_prev_R = 0;
bool cugo_direction_L; //true:forward false:backward
bool cugo_direction_R; //true:forward false:backward

long int cugo_target_count_L = 0;
long int cugo_target_count_R = 0;
bool cugo_button_check = false;
int cugoRunMode = CUGO_ARDUINO_MODE;

//プロポ入力関連
int OLD_PWM_IN_PIN0_VALUE;   // プロポスティック入力値(L)
int OLD_PWM_IN_PIN1_VALUE;   // プロポスティック入力値(MODE)
int OLD_PWM_IN_PIN2_VALUE;   // プロポスティック入力値(R)
int OLD_CMD_BUTTON_VALUE = 0; 
volatile unsigned long upTime[PWM_IN_MAX];
volatile unsigned long cugoRcTime[PWM_IN_MAX];
volatile unsigned long time[PWM_IN_MAX];
volatile unsigned long long cugoButtonTime;


/*
MotorController cugo_motor_controllers[MOTOR_NUM] = {
  MotorController(PIN_ENCODER_L_A, PIN_ENCODER_L_B, PIN_MOTOR_L, 2048, 600, 100, L_LPF, L_KP, L_KI, L_KD, L_reverse),
  MotorController(PIN_ENCODER_R_A, PIN_ENCODER_R_B, PIN_MOTOR_R, 2048, 600, 100, R_LPF, R_KP, R_KI, R_KD, R_reverse)
};
*/

// LEFTインスタンス有効化
//  cugo_motor_controllers[MOTOR_LEFT] = MotorController(PIN_ENCODER_L_A, PIN_ENCODER_L_B, PIN_MOTOR_L, 2048, 600, 100, L_LPF, L_KP, L_KI, L_KD, L_reverse);
// RIGHTインスタンス有効化
//  cugo_motor_controllers[MOTOR_RIGHT] = MotorController(PIN_ENCODER_R_A, PIN_ENCODER_R_B, PIN_MOTOR_R, 2048, 600, 100, R_LPF, R_KP, R_KI, R_KD, R_reverse);

/*-----------------------------------------------*/
/*MiddleUser向け関数*/
void cugo_init(){
  init_display();
  init_KOPROPO(OLD_PWM_IN_PIN0_VALUE,OLD_PWM_IN_PIN1_VALUE,OLD_PWM_IN_PIN2_VALUE);
  cugo_reset_button_times();

  pinMode(PIN_ENCODER_L_A, INPUT_PULLUP);     //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  pinMode(PIN_ENCODER_L_B, INPUT_PULLUP);     //B相用信号入力　内蔵プルアップ有効
  pinMode(PIN_ENCODER_R_A, INPUT_PULLUP);     //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  pinMode(PIN_ENCODER_R_B, INPUT_PULLUP);     //B相用信号入力　内蔵プルアップ有効
    
}

//モード切り替わり確認
void cugo_check_mode_change(MotorController cugo_motor_controllers[MOTOR_NUM])
{
  noInterrupts();      //割り込み停止
  cugoRcTime[0] = time[0];
  cugoRcTime[1] = time[1];
  cugoRcTime[2] = time[2];
  cugoButtonTime = time[3];
  interrupts();     //割り込み開始
  
  //Serial.print(F("cugo : old =  "));
  //Serial.print(String(cugoRunMode));
  //Serial.println(String(cugo_old_runmode));
  if ((cugoRunMode == CUGO_ARDUINO_MODE) && (cugo_old_runmode == CUGO_RC_MODE))
  {
    Serial.println(F("### モード:CUGO_ARDUINO_MODE ###"));
    digitalWrite(LED_BUILTIN, HIGH);  // CUGO_ARDUINO_MODEでLED点灯           
    cugo_old_runmode = CUGO_ARDUINO_MODE;
    reset_pid_gain(cugo_motor_controllers);
    cugo_motor_direct_instructions(1500, 1500,cugo_motor_controllers); //直接停止命令を出す
    delay(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。    
  }
  if(cugoRunMode == CUGO_RC_MODE && cugo_old_runmode == CUGO_ARDUINO_MODE)
  {
    Serial.println(F("###   モード:CUGO_RC_MODE    ###"));
    digitalWrite(LED_BUILTIN, LOW); // CUGO_RC_MODEでLED消灯
    cugo_old_runmode = CUGO_RC_MODE;            
    reset_pid_gain(cugo_motor_controllers);
    cugo_motor_direct_instructions(1500, 1500,cugo_motor_controllers); //直接停止命令を出す
    delay(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。    
  }                       
}
void cugo_keep_speed_ms(unsigned long int wait_ms,MotorController cugo_motor_controllers[MOTOR_NUM]){ 
//void cugo_keep_speed_ms(unsigned long int wait_ms){ 
  //wait値の範囲は0から4,294,967,295micors 最大71.58278825分//★これでよいか？
  unsigned long long int cugo_target_wait_time = micros()+ wait_ms*1000;
  //Serial.print("wait_ms::"+String(cugo_target_wait_time/1000));
  unsigned long long int cugo_current_time = micros();
  //Serial.print("," + String(cugo_current_time));
  while(cugo_target_wait_time > cugo_current_time){
    for (int i = 0; i < MOTOR_NUM; i++) { 
      cugo_motor_controllers[i].driveMotor();      
    }
    cugo_calc_odometer(cugo_motor_controllers);    
    
    cugo_current_time = micros();
  }
  //Serial.println(":: wait done!" + String(cugo_current_time/1000));
}
void cugo_keep_stop_ms(unsigned long int wait_ms,MotorController cugo_motor_controllers[MOTOR_NUM]){
  //wait値の範囲は0から4,294,967,295micors 最大71.58278825分//★これでよいか？//追加時間分
  cugo_stop(cugo_motor_controllers);
  unsigned long long int cugo_target_wait_time = micros()+ wait_ms*1000;
  //Serial.print("wait_ms::"+String(cugo_target_wait_time/1000));
  unsigned long long int cugo_current_time = micros();
  //Serial.print("," + String(cugo_current_time));
  while(cugo_target_wait_time > cugo_current_time){
    cugo_current_time = micros();
  }
  cugo_stop(cugo_motor_controllers);
}

void cugo_move_pid(float target_rpm,bool use_pid,MotorController cugo_motor_controllers[MOTOR_NUM]){
  reset_pid_gain(cugo_motor_controllers);          
  float l_count_p =0 ;  // P制御値
  float l_count_i =0 ;  // I制御値    
  float l_count_d =0 ;  // D制御値
  float r_count_p =0 ;  // P制御値
  float r_count_i =0 ;  // I制御値
  float r_count_d =0 ;  // D制御値

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
       cugo_motor_controllers[MOTOR_LEFT].setTargetRpm(target_rpm);
    }else{
      cugo_motor_controllers[MOTOR_LEFT].setTargetRpm(-target_rpm);
    }
    if(cugo_direction_R){
       cugo_motor_controllers[MOTOR_RIGHT].setTargetRpm(target_rpm);
    }else{
      cugo_motor_controllers[MOTOR_RIGHT].setTargetRpm(-target_rpm);
    }
  }
  
  // PID位置制御のデータ格納
  float l_count_prev_i_ =0 ;
  float l_count_prev_p_ =0 ;
  //float l_count_prev_p_ = (cugo_target_count_L - cugo_motor_controllers[MOTOR_LEFT].getCount())/10000.0;
  float r_count_prev_i_ =0 ;
  float r_count_prev_p_ =0 ;
  //float r_count_prev_p_ = (cugo_target_count_R - cugo_motor_controllers[MOTOR_RIGHT].getCount())/10000.0 ;
  float l_count_gain =0 ;
  float r_count_gain =0 ;


  Serial.println("===========");  

  //ここでcugo_target_count_L,cugo_target_count_Rを絶対値に変更
  //cugo_target_count_L = cugo_motor_controllers[MOTOR_LEFT].getCount()+cugo_target_count_L;
  //cugo_target_count_R = cugo_motor_controllers[MOTOR_RIGHT].getCount()+cugo_target_count_R;
  Serial.println("start l:r:" + String(cugo_motor_controllers[MOTOR_LEFT].getCount())+" ,"+ String(cugo_motor_controllers[MOTOR_RIGHT].getCount()));  
  Serial.println("target l:r:" + String(cugo_target_count_L)+" ,"+ String(cugo_target_count_R));    

  while(!cugo_check_count_achivement(MOTOR_LEFT,cugo_motor_controllers) || !cugo_check_count_achivement(MOTOR_RIGHT,cugo_motor_controllers)){  
      if(cugo_target_count_L == 0 && cugo_target_count_R == 0)
      {
        //停止しているだけの時
        cugo_motor_controllers[MOTOR_LEFT].setTargetRpm(0);
        cugo_motor_controllers[MOTOR_RIGHT].setTargetRpm(0);

      } else{
        if(use_pid){
          // 各制御値の計算
          l_count_p = (cugo_target_count_L - cugo_motor_controllers[MOTOR_LEFT].getCount())/10000.0;
          l_count_i = l_count_prev_i_ + l_count_p;
          l_count_d = l_count_p - l_count_prev_p_;
          r_count_p = (cugo_target_count_R - cugo_motor_controllers[MOTOR_RIGHT].getCount())/10000.0;
          r_count_i = r_count_prev_i_ + r_count_p;
          r_count_d = r_count_p - r_count_prev_p_;

          l_count_i = min( max(l_count_i,-L_MAX_COUNT_I),L_MAX_COUNT_I);        
          r_count_i = min( max(r_count_i,-R_MAX_COUNT_I),R_MAX_COUNT_I);
          // PID制御
          l_count_gain = (l_count_p * L_COUNT_KP + l_count_i * L_COUNT_KI + l_count_d * L_COUNT_KD);  
          r_count_gain = (r_count_p * R_COUNT_KP + r_count_i * R_COUNT_KI + r_count_d * R_COUNT_KD);  
          // prev_ 更新
          l_count_prev_p_ = l_count_p;
          l_count_prev_i_ = l_count_i;
          r_count_prev_p_ = r_count_p;
          r_count_prev_i_ = r_count_i;
          Serial.print("pidgain::" + String(l_count_p * L_COUNT_KP)+" ,"+String(l_count_i * L_COUNT_KI)+" ,"+String(l_count_d * L_COUNT_KD));    
          Serial.println("  gain::" + String(l_count_gain));    
          l_count_gain = min( max(l_count_gain,-MAX_MOTOR_RPM),MAX_MOTOR_RPM);//モーターの速度上限        
          r_count_gain = min( max(r_count_gain,-MAX_MOTOR_RPM),MAX_MOTOR_RPM);//モーターの速度上限             
          l_count_gain = min( max(l_count_gain,-fabsf(target_rpm)),fabsf(target_rpm));//ユーザ設定の速度上限        
          r_count_gain = min( max(r_count_gain,-fabsf(target_rpm)),fabsf(target_rpm));//ユーザ設定の速度上限  
   
          //位置制御
          if(!cugo_check_count_achivement(MOTOR_LEFT,cugo_motor_controllers)){
          cugo_motor_controllers[MOTOR_LEFT].setTargetRpm(l_count_gain);          
          } 
          if(!cugo_check_count_achivement(MOTOR_RIGHT,cugo_motor_controllers)){
          cugo_motor_controllers[MOTOR_RIGHT].setTargetRpm(r_count_gain);
          }

       }
    }
    for (int i = 0; i < MOTOR_NUM; i++){ 
      cugo_motor_controllers[i].driveMotor();
    }
     delay(10);
     //cugo_keep_speed_ms(10,cugo_motor_controllers);
    
    cugo_calc_odometer(cugo_motor_controllers);    
  }
  Serial.println("result l:r:" + String(cugo_motor_controllers[MOTOR_LEFT].getCount())+" ,"+ String(cugo_motor_controllers[MOTOR_RIGHT].getCount()));
  //Serial.println("距離："+String(conversion_count_to_distance*(cugo_motor_controllers[0].getCount())));
  cugo_stop(cugo_motor_controllers); 
  
  reset_pid_gain(cugo_motor_controllers);                 
}
  //前進制御＆回転制御
  //目標距離に前進または後進　位置制御あり
void cugo_move_forward(float target_distance,MotorController cugo_motor_controllers[MOTOR_NUM]){
  calc_necessary_count(target_distance,cugo_motor_controllers);
  cugo_move_pid(NORMAL_MOTOR_RPM,true,cugo_motor_controllers);
  }
void cugo_move_forward(float target_distance,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]){
  calc_necessary_count(target_distance,cugo_motor_controllers);
  cugo_move_pid(target_rpm,true,cugo_motor_controllers);
  }
void cugo_move_forward_raw(float target_distance,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]){
  calc_necessary_count(target_distance,cugo_motor_controllers);
  cugo_move_pid(target_rpm,false,cugo_motor_controllers);
  }
  //目標角度に回転　位置制御あり
void cugo_turn_clockwise(float target_degree,MotorController cugo_motor_controllers[MOTOR_NUM]){
  calc_necessary_rotate(target_degree,cugo_motor_controllers);
  
  cugo_move_pid(NORMAL_MOTOR_RPM,true,cugo_motor_controllers);  
  }
void cugo_turn_clockwise(float target_degree,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]){
  calc_necessary_rotate(target_degree,cugo_motor_controllers);
  cugo_move_pid(target_rpm,true,cugo_motor_controllers);
  }
void cugo_turn_clockwise_raw(float target_degree,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]){
  calc_necessary_rotate(target_degree,cugo_motor_controllers);
  cugo_move_pid(target_rpm,false,cugo_motor_controllers);
  }
void cugo_turn_counterclockwise(float target_degree,MotorController cugo_motor_controllers[MOTOR_NUM]){
  calc_necessary_rotate(-target_degree,cugo_motor_controllers);
  cugo_move_pid(NORMAL_MOTOR_RPM,true,cugo_motor_controllers);    
  }
void cugo_turn_counterclockwise(float target_degree,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]){
  calc_necessary_rotate(-target_degree,cugo_motor_controllers);
  cugo_move_pid(target_rpm,true,cugo_motor_controllers);  
  }
void cugo_turn_counterclockwise_raw(float target_degree,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]){
  calc_necessary_rotate(-target_degree,cugo_motor_controllers);
  cugo_move_pid(target_rpm,false,cugo_motor_controllers);  
  }
  //極座標での移動命令
void cugo_polar_coordinates_theta_raw(float target_radius,float target_theta,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]){
  cugo_target_count_L = (target_radius-tread/2)*target_theta*conversion_distance_to_count;
  cugo_target_count_R = (target_radius+tread/2)*target_theta*conversion_distance_to_count;      
  cugo_motor_controllers[MOTOR_LEFT].setTargetRpm(target_rpm*((target_radius-tread/2)/target_radius));
  cugo_motor_controllers[MOTOR_RIGHT].setTargetRpm(target_rpm*((target_radius+tread/2)/target_radius));
  while(cugo_check_count_achivement(MOTOR_LEFT,cugo_motor_controllers) && cugo_check_count_achivement(MOTOR_RIGHT,cugo_motor_controllers)){  
      if(cugo_target_count_L == 0 && cugo_target_count_R == 0)
      {
        //停止しているだけの時
        cugo_motor_controllers[MOTOR_LEFT].setTargetRpm(0);
        cugo_motor_controllers[MOTOR_RIGHT].setTargetRpm(0);
      }
      for (int i = 0; i < MOTOR_NUM; i++){ 
        cugo_motor_controllers[i].driveMotor();
      }
      cugo_calc_odometer(cugo_motor_controllers);    
    }  
  cugo_stop(cugo_motor_controllers); 
  reset_pid_gain(cugo_motor_controllers);          
  }
void cugo_polar_coordinates_distance_raw(float target_radius,float target_disttance,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]){
  cugo_target_count_L = target_disttance*((target_radius-tread/2)/target_radius)*conversion_distance_to_count;
  cugo_target_count_R = target_disttance*((target_radius+tread/2)/target_radius)*conversion_distance_to_count;      
  cugo_motor_controllers[MOTOR_LEFT].setTargetRpm(target_rpm*((target_radius-tread/2)/target_radius));
  cugo_motor_controllers[MOTOR_RIGHT].setTargetRpm(target_rpm*((target_radius+tread/2)/target_radius));
  while(cugo_check_count_achivement(MOTOR_LEFT,cugo_motor_controllers) && cugo_check_count_achivement(MOTOR_RIGHT,cugo_motor_controllers)){  
      if(cugo_target_count_L == 0 && cugo_target_count_R == 0)
      {
        //停止しているだけの時
        cugo_motor_controllers[MOTOR_LEFT].setTargetRpm(0);
        cugo_motor_controllers[MOTOR_RIGHT].setTargetRpm(0);
      }
      for (int i = 0; i < MOTOR_NUM; i++){ 
        cugo_motor_controllers[i].driveMotor();
      }
      cugo_calc_odometer(cugo_motor_controllers);    
    }  
  cugo_stop(cugo_motor_controllers); 
  reset_pid_gain(cugo_motor_controllers);          
  }
  //チェック関連
bool cugo_check_count_achivement(int motor_num_,MotorController cugo_motor_controllers[MOTOR_NUM]){
    long int target_count_ = 0;
    bool cugo_direction_;
    if(motor_num_ == MOTOR_LEFT){
      target_count_ = cugo_target_count_L;
      cugo_direction_ = cugo_direction_L;
    }else if(motor_num_ == MOTOR_RIGHT){
      target_count_ = cugo_target_count_R;
      cugo_direction_ = cugo_direction_R;      
    }else{
    return false;
    }
    
    // 目標達成チェック
    //Serial.print("count::"String(cugo_target_count_L));
    if(cugo_direction_){
      if (target_count_<cugo_motor_controllers[motor_num_].getCount()){
        cugo_motor_controllers[motor_num_].setTargetRpm(0);
        return true;
      } 
    }else{
      if (target_count_>cugo_motor_controllers[motor_num_].getCount()){
        cugo_motor_controllers[motor_num_].setTargetRpm(0);
        return true;
      } 
    }

    return false;
}
int cugo_check_a_channel_value(){
  noInterrupts();      //割り込み停止
  cugoRcTime[0] = time[0];
  interrupts();     //割り込み開始  
  return cugoRcTime[0];
} 
int cugo_check_b_channel_value(){
  noInterrupts();      //割り込み停止
  cugoRcTime[1] = time[1];
  interrupts();     //割り込み開始  
  return cugoRcTime[1];
} 
int cugo_check_c_channel_value(){
  noInterrupts();      //割り込み停止
  cugoRcTime[2] = time[2];
  interrupts();     //割り込み開始  
  return cugoRcTime[2];
} 
bool cugo_check_button(){
  return cugo_button_check;  //いる？か
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
    PIN_DOWN(3);
    cugoButtonTime = time[3];
  interrupts();     //割り込み開始
    return cugoButtonTime/1000;  
  }else{
    return 0;      
  }


}
float cugo_check_odometer(){
  return cugo_odometer;  
}
void cugo_calc_odometer(MotorController cugo_motor_controllers[MOTOR_NUM]){
  cugo_odometer = fabsf(cugo_odometer+((cugo_motor_controllers[MOTOR_LEFT].getCount()-cugo_count_prev_L)+(cugo_count_prev_R - cugo_motor_controllers[MOTOR_RIGHT].getCount())/2)*conversion_count_to_distance);
  cugo_count_prev_L = cugo_motor_controllers[MOTOR_LEFT].getCount();
  cugo_count_prev_R = cugo_motor_controllers[MOTOR_RIGHT].getCount();  
}
void cugo_reset_odometer(int check_number){
  cugo_odometer = 0.0;  
}
void init_KOPROPO(int OLD_PWM_IN_PIN0_VALUE,int OLD_PWM_IN_PIN1_VALUE,int OLD_PWM_IN_PIN2_VALUE)
{
  //Serial.println(F("#   init_KOPROPO"));//確認用
  // ピン変化割り込みの初期状態保存
  cugoRunMode = CUGO_RC_MODE;
  OLD_PWM_IN_PIN0_VALUE = digitalRead(PWM_IN_PIN0);
  OLD_PWM_IN_PIN1_VALUE = digitalRead(PWM_IN_PIN1);
  OLD_PWM_IN_PIN2_VALUE = digitalRead(PWM_IN_PIN2);
  OLD_PWM_IN_PIN2_VALUE = digitalRead(CMD_BUTTON_PIN);
  
  // ピン変化割り込みの設定（D5,D6,D7をレジスタ直接読み取りで割り込み処理）
  pinMode(PWM_IN_PIN0, INPUT);
  pinMode(PWM_IN_PIN1, INPUT);
  pinMode(PWM_IN_PIN2, INPUT);
  pinMode(CMD_BUTTON_PIN, INPUT_PULLUP);
  
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
  delay(100);
}
void calc_necessary_rotate(float degree,MotorController cugo_motor_controllers[MOTOR_NUM]) 
{
  //Serial.println(F("#   calc_necessary_rotate"));//確認用
//  cugo_target_count_L = cugo_motor_controllers[MOTOR_LEFT].getCount();
//  cugo_target_count_R = cugo_motor_controllers[MOTOR_RIGHT].getCount();
  cugo_target_count_L = ((degree / 360) * tread * PI) * encoder_resolution / (2 * wheel_radius_l * PI);
  cugo_target_count_R = -((degree / 360) * tread * PI) * encoder_resolution / (2 * wheel_radius_r * PI);
  //Serial.println("degree: " + String(degree));
  //Serial.println("### cugo_target_count_L/R: " + String(*cugo_target_count_L) + " / " + String(*cugo_target_count_R) + "###");
  //Serial.println("kakudo: " + String((degree / 360) * tread * PI));
  //Serial.println("PI: " + String(PI));
  //Serial.println("issyuu: " + String(2 * wheel_radius_r * PI));
}
void calc_necessary_count(float distance,MotorController cugo_motor_controllers[MOTOR_NUM]) 
{
  //Serial.println(F("#   calc_necessary_count"));//確認用
  //  cugo_target_count_L = distance * encoder_resolution / (2 * wheel_radius_l * PI);
  //  cugo_target_count_R = distance * encoder_resolution / (2 * wheel_radius_r * PI);

  //cugo_target_count_L = distance / (2 * wheel_radius_l * PI);
  //cugo_target_count_R = distance / (2 * wheel_radius_r * PI);
  //cugo_target_count_L = cugo_target_count_L * encoder_resolution;
  //cugo_target_count_R = cugo_target_count_R * encoder_resolution;
  //cugo_target_count_L = cugo_motor_controllers[MOTOR_LEFT].getCount();
  //cugo_target_count_R = cugo_motor_controllers[MOTOR_RIGHT].getCount();
  cugo_target_count_L = distance * conversion_distance_to_count;
  cugo_target_count_R = distance * conversion_distance_to_count;
  //Serial.println("distance: " + String(distance));
  //Serial.println("distance: " + String(encoder_resolution));
  //Serial.println("2 * wheel_radius_l * PI: " + String(2 * wheel_radius_l * PI));
  //Serial.println("calc: " + String(distance * encoder_resolution / (2 * wheel_radius_l * PI)));

  //Serial.println("### cugo_target_count_L/R: " + String(*cugo_target_count_L) + " / " + String(*cugo_target_count_R) + "###");
  //Serial.println("distance: " + String(distance));
  //Serial.println("wheel_radius_l: " + String(wheel_radius_l));
  //Serial.println("PI: " + String(PI));
  //Serial.println("issyuu: " + String(2 * wheel_radius_r * PI));

}
void cugo_motor_direct_instructions(int left, int right,MotorController cugo_motor_controllers[MOTOR_NUM])// cugo_motor_controllers[0] MOTOR_LEFT cugo_motor_controllers[1] MOTOR_RIGHT
{
  //Serial.println(F("#   cugo_motor_direct_instructions"));//確認用
  cugo_motor_controllers[0].servo_.writeMicroseconds(left);
  cugo_motor_controllers[1].servo_.writeMicroseconds(right);
}
void cugo_rcmode(volatile unsigned long cugoRcTime[PWM_IN_MAX],MotorController cugo_motor_controllers[MOTOR_NUM])
{
  //Serial.println(F("#   cugo_CUGO_RC_MODE"));//確認用  
  //digitalWrite(LED_BUILTIN, LOW); // CUGO_RC_MODEでLED消灯
  // 値をそのままへESCへ出力する
  cugo_motor_direct_instructions(cugoRcTime[0], cugoRcTime[2],cugo_motor_controllers);
  //Serial.println("input cmd:" + String(cugoRcTime[0]) + ", " + String(cugoRcTime[2]));
}

void cugo_stop(MotorController cugo_motor_controllers[MOTOR_NUM])
{
  //Serial.println(F("#   cugo_stop"));//確認用
  cugo_motor_controllers[0].setTargetRpm(0.0);
  cugo_motor_controllers[1].setTargetRpm(0.0);
  cugo_motor_direct_instructions(1500, 1500,cugo_motor_controllers);
  delay(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。    
  
}
void reset_pid_gain(MotorController cugo_motor_controllers[MOTOR_NUM])
{
  //Serial.println(F("#   reset_pid_gain"));//確認用  
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    cugo_motor_controllers[i].reset_PID_param();
  }
}
void init_display()
{
  delay(30);
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
  Serial.println("###   CugoAruduinoKit起動  　###");
  Serial.println("###############################");  

}
