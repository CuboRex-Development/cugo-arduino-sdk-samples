// CugoArduinoMiffdleUserProgramming用ライブラリ
#ifndef CUGOARDUINOMODE_H
#define CUGOARDUINOMODE_H

#include "MotorController.h"

// モータとエンコーダのピン配置設定
#define PIN_MOTOR_L A0  // モータ出力ピン(L)
#define PIN_MOTOR_R A1  // モータ出力ピン(R)
#define PIN_ENCODER_L_A 2  // エンコーダ割り込み入力ピン(L)
#define PIN_ENCODER_L_B 8  // エンコーダ回転方向入力ピン(L)
#define PIN_ENCODER_R_A 3  // エンコーダ割り込み入力ピン(R)
#define PIN_ENCODER_R_B 9  // エンコーダ回転方向入力ピン(R)

// プロポ信号の読み取りピン（L/R/MODE_CAHNGE）
#define PWM_IN_PIN0   5   // プロポスティック入力ピン(L)//digitalRead ピン変化割り込みの設定
#define PWM_IN_PIN1   6   // プロポスティック入力ピン(MODE)//digitalRead ピン変化割り込みの設定
#define PWM_IN_PIN2   7   // プロポスティック入力ピン(R)//digitalRead ピン変化割り込みの設定

//cugo仕様関連
#define wheel_radius_l  0.03858d
#define wheel_radius_r  0.03858d
#define tread  0.380d
#define encoder_resolution  2048.0d 
#define MAX_MOTOR_RPM 180 //モータの速度上限値
//↑の仕様が変更される場合は下の変換係数も変更してください。
#define conversion_distance_to_count 8448.660535308492d // 変換係数： encoder_resolution / (2 * wheel_radius_l * PI)の計算結果
#define conversion_count_to_distance 0.000118361958d    // 変換係数： 2 * wheel_radius_l * PI  / encoder_resolution の計算結果

// PID ゲイン調整
// L側
#define L_KP   1.0f   //CuGoV3
#define L_KI   0.02f   //CuGoV3
#define L_KD   0.1f   //CuGoV3

//const float L_KP = 1.0;
//const float L_KI = 0.06;
//const float L_KD = 0.1;

// R側
#define R_KP   1.0f   //CuGoV3
#define R_KI   0.02f   //CuGoV3
#define R_KD   0.1f   //CuGoV3
//const float R_KP = 1.0;
//const float R_KI = 0.06;
//const float R_KD = 0.1;

// ローパスフィルタ
#define L_LPF   0.2f 
#define R_LPF   0.2f
//const float L_LPF = 0.2;
//const float R_LPF = 0.2;

/*
// PID位置制御のゲイン調整
#define L_COUNT_KP  0.04f
#define L_COUNT_KI  0.003f 
#define L_COUNT_KD  0.01f
#define R_COUNT_KP  0.04f
#define R_COUNT_KI  0.003f 
#define R_COUNT_KD  0.01f

#define L_MAX_COUNT_I  9000.0f //速度上限を設定している場合はiは必ず0に
#define R_MAX_COUNT_I  9000.0f //速度上限を設定している場合はiは必ず0に
*/

// PID位置制御のゲイン調整

#define L_COUNT_KP  50.0f
#define L_COUNT_KI  0.5f 
#define L_COUNT_KD  10.0f
#define R_COUNT_KP  50.0f
#define R_COUNT_KI  0.5f 
#define R_COUNT_KD  10.0f

#define L_MAX_COUNT_I  120 
#define R_MAX_COUNT_I  120 

// Arduinoキットのスタートボタン
#define CMD_BUTTON_PIN A2 

// 動作モード定義
#define CUGO_RC_MODE 0
#define CUGO_ARDUINO_MODE 1

#define cugo_propo_Ach 0
#define cugo_propo_Bch 1
#define cugo_propo_Cch 2

//各種閾値
#define CUGO_PROPO_MAX   1900  // ARDUINOモードに入るときの閾値(us) (1100~1900/中央1500)
#define CUGO_PROPO_MIN   1100  // ARDUINOモードに入るときの閾値(us) (1100~1900/中央1500)
#define CUGO_ARDUINO_MODE_IN   1700  // ARDUINOモードに入るときの閾値(us) (1100~1900/中央1500)
#define CUGO_ARDUINO_MODE_OUT  1300  // ARDUINOモードから抜けるときの閾値(us) (1100~1900/中央1500)
#define EXCEPTION_NO -32768 //int下限
#define NORMAL_MOTOR_RPM 50

//モーター設定
#define MOTOR_NUM 2 // モータ接続数（最大4の予定）
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

//PIN関連 
#define PIN_UP(no)    upTime[no] = micros();
#define PIN_DOWN(no)  time[no] = micros() - upTime[no]
#define PWM_IN_MAX    4
#define CUGO_BUTTON_CHECK_BORDER 50000
// グローバル変数宣言

extern int cugo_old_runmode;
extern int cugo_button_count;
extern float cugo_odometer;
extern long int cugo_count_prev_L;
extern long int cugo_count_prev_R;


extern long int cugo_target_count_L;
extern long int cugo_target_count_R;
extern long int cugo_odometer_count_theta;
extern long int cugo_odometer_count_x;
extern long int cugo_odometer_count_y;
extern int cugoRunMode;
extern const bool CUGO_L_reverse;
extern const bool CUGO_R_reverse;
extern bool cugo_direction_L; //true:forward false:backward
extern bool cugo_direction_R; //true:forward false:backward
extern bool cugo_button_check;
extern bool cugo_button_check;
extern int OLD_CMD_BUTTON_VALUE; 
extern int OLD_PWM_IN_PIN0_VALUE; 
extern int OLD_PWM_IN_PIN1_VALUE; 
extern int OLD_PWM_IN_PIN2_VALUE; 
extern volatile unsigned long upTime[PWM_IN_MAX];
extern volatile unsigned long cugoRcTime[PWM_IN_MAX];
extern volatile unsigned long long cugoButtonTime;
extern volatile unsigned long time[PWM_IN_MAX];

//extern MotorController cugo_motor_controllers[MOTOR_NUM];


//各種関数
  void init_display();
  void init_KOPROPO(int OLD_PWM_IN_PIN0_VALUE,int OLD_PWM_IN_PIN1_VALUE,int OLD_PWM_IN_PIN2_VALUE);
  void calc_necessary_rotate(float degree,MotorController cugo_motor_controllers[MOTOR_NUM]); 
  void calc_necessary_count(float distance,MotorController cugo_motor_controllers[MOTOR_NUM]); 
  void reset_pid_gain(MotorController cugo_motor_controllers[MOTOR_NUM]);

/*-----------------------------------------------*/
/*MiddleUser向け関数*/ //★motorclassのたんなるラップアップは不要
  //初期設定関数関連
  void cugo_init();
  void cugo_check_mode_change(MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cugo_keep_speed_ms(unsigned long int wait_ms,MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cugo_keep_stop_ms(unsigned long int wait_ms,MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cugo_wait(unsigned long int wait_ms);
  void cugo_long_wait(unsigned long int wait_seconds);
  void cugo_motor_direct_instructions(int left, int right,MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cugo_rcmode(volatile unsigned long cugoRcTime[PWM_IN_MAX],MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cugo_stop(MotorController cugo_motor_controllers[MOTOR_NUM]);


  //前進制御＆回転制御
  //目標距離に前進または後進　位置制御あり
  void cugo_move_forward(float target_distance,MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cugo_move_forward(float target_distance,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]);//単位はm,rpm
  void cugo_move_forward_raw(float target_distance,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]);//単位はm,rpm
  //目標角度に回転　位置制御あり
  void cugo_turn_clockwise(float target_degree,MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cugo_turn_clockwise(float target_degree,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]);//単位はm,rpm
  void cugo_turn_clockwise_raw(float target_degree,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]);//単位はm,rpm
  void cugo_turn_counterclockwise(float target_degree,MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cugo_turn_counterclockwise(float target_degree,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]);//単位はm,rpm
  void cugo_turn_counterclockwise_raw(float target_degree,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]);//単位はm,rpm  
  //極座標での移動命令
  void cugo_curve_theta_raw(float target_radius,float target_theta,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cugo_curve_distance_raw(float target_radius,float target_distance,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]);

  //カウント数のチェック
  bool cugo_check_count_achievement(int motor_num_,MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cugo_move_pid(float target_rpm,bool use_pid,MotorController cugo_motor_controllers[MOTOR_NUM]);//単位はm,rpm
  //モーター制御
  int cugo_check_propo_channel_value(int channel_number);
  int cugo_check_a_channel_value(); //現状のachの値取得
  int cugo_check_b_channel_value(); //現状のbchの値取得
  int cugo_check_c_channel_value(); //現状のcchの値取得
  
  bool cugo_check_button(); //現状の押されているか
  int cugo_check_button_times(); //現状の押された回数
  void cugo_reset_button_times(); //現状の押された回数
  long int cugo_button_press_time(); //ボタンの押されている時間
  //オドメトリ計算
  float cugo_check_odometer(int check_number,MotorController cugo_motor_controllers[MOTOR_NUM]); 
  void cugo_calc_odometer(MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cugo_reset_odometer(int check_number);

//void cugo_setup();
//void cugoLeftEncHandler();
//void cugoRightEncHandler();


  //以下よく使うであろうMotorController
  /*
   * driveMotor():モーターへサーボ入力　入力値は事前に設定されたsetTargetRpmの値とPID速度制御から算出
   * reset_PID_param() PIDのパラメータを初期化
   * setTargetRpm(float target_rpm) 目標ＲＰＭを設定
   * getTargetRpm()　目標RPM値を取得
   * getCount()　現状のカウント数を把握
   * getRpm()　現状のRPM値を取得　getrpmの前にcalcRpm()をしておくとより正確
   * getSpeed() 現状の速度を取得
  
  */
  
  
#endif
