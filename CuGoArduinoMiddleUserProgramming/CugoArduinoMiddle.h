// CugoArduinoMiddle.h
// CugoArduinoMiffdleUserProgramming用ライブラリ

#ifndef CUGOARDUINOMODE_H
#define CUGOARDUINOMODE_H

#include "Arduino.h"
#include <SPI.h>
#include <Servo.h>
#include "MotorController.h"
#include "CugoArduinoMiddle.h"

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

//encoder_resolution / (2 * wheel_radius_l * PI);

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

// PID位置制御のゲイン調整
#define L_COUNT_KP  0.04f
#define L_COUNT_KI  0.003f 
#define L_COUNT_KD  0.01f
#define R_COUNT_KP  0.04f
#define R_COUNT_KI  0.003f 
#define R_COUNT_KD  0.01f

#define L_MAX_COUNT_I  9000.0f //速度上限を設定している場合はiは必ず0に
#define R_MAX_COUNT_I  9000.0f //速度上限を設定している場合はiは必ず0に


#define CONTROLL_STOP_count  1000

// Arduinoキットのスタートボタン
#define CMD_BUTTON_PIN A2 


// 動作モード定義
#define CUGO_RC_MODE 0
#define CUGO_ARDUINO_MODE 1

//各種閾値
#define CUGO_ARDUINO_MODE_IN   1700  // ARDUINOモードに入るときの閾値(us) (1100~1900/中央1500)
#define CUGO_ARDUINO_MODE_OUT  1300  // ARDUINOモードから抜けるときの閾値(us) (1100~1900/中央1500)
#define CMD_SIZE 60 //　コマンド数上限
#define EXCEPTION_NO -32768 //int下限

//モーター設定
#define MOTOR_NUM 2 // モータ接続数（最大4の予定）
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

//PIN関連 //★ここはユーザがいじらないことをルール決め
#define PIN_UP(no)    upTime[no] = micros();
#define PIN_DOWN(no)  time[no] = micros() - upTime[no]
#define PWM_IN_MAX    4
#define CUGO_BUTTON_CHECK_BORDER 50000
// グローバル変数宣言

//
//extern bool cugo_Bch_flag;//true:BchCUGO_ARDUINO_MODE_IN～CUGO_ARDUINO_MODE_OUTの間に入った
extern bool cugo_button_flag;//true:BchCUGO_ARDUINO_MODE_IN～CUGO_ARDUINO_MODE_OUTの間に入った
extern int oldRunMode;
extern int cugo_button_count;


extern long int arduino_count_cmd_matrix[CMD_SIZE][2];
extern int arduino_flag_cmd_matrix[CMD_SIZE][4];
extern int init_current_cmd;

extern long int cugo_target_count_L;
extern long int cugo_target_count_R;
extern long int target_wait_time;
extern int button_push_count;
extern bool button_enable;
extern bool cmd_init;
extern int current_cmd;
extern bool cmd_L_back;
extern bool cmd_R_back;
extern bool cmd_exec;
extern bool count_done;
extern bool wait_done;
extern bool button_done;
extern bool spi_done;
extern bool end_arduino_mode;
extern unsigned long long current_time;
extern unsigned long long prev_time_10ms; 
extern unsigned long long prev_time_100ms; 
extern unsigned long long prev_time_1000ms; 
extern int cugoRunMode;
extern bool UDP_CONNECTION_DISPLAY;
extern bool ENCODER_DISPLAY;
extern bool PID_CONTROLL_DISPLAY;
extern bool FAIL_SAFE_DISPLAY;
extern const bool L_reverse;
extern const bool R_reverse;
extern bool button_check;
extern int OLD_CMD_BUTTON_VALUE; 
extern int OLD_PWM_IN_PIN0_VALUE; 
extern int OLD_PWM_IN_PIN1_VALUE; 
extern int OLD_PWM_IN_PIN2_VALUE; 
extern volatile unsigned long upTime[PWM_IN_MAX];
extern volatile unsigned long cugoRcTime[PWM_IN_MAX];
extern volatile unsigned long long cugoButtonTime;
extern volatile unsigned long time[PWM_IN_MAX];



//各種関数
  void init_display();
  void init_SPI();
  void init_KOPROPO(int OLD_PWM_IN_PIN0_VALUE,int OLD_PWM_IN_PIN1_VALUE,int OLD_PWM_IN_PIN2_VALUE);
  void init_ARDUINO_CMD();
  void set_arduino_cmd_matrix(long int cmd_0,long  int cmd_1, int cmd_2, int cmd_3,int cmd_4,int cmd_5);
  void send_spi(int mode);
  void view_arduino_cmd_matrix();
  void display_failsafe(bool FAIL_SAFE_DISPLAY,int cugoRunMode);
  void display_nothing();
  void spi_cmd(int spi_cmd_value);
  void calc_necessary_rotate(float degree); 
  void calc_necessary_count(float distance); 
  void atamaopen();
  void atamaclose();
  void wait_button();
  void botan();
  void button();
  void display_speed(MotorController cugo_motor_controllers[MOTOR_NUM],bool ENCODER_DISPLAY); 
  void display_target_rpm(MotorController cugo_motor_controllers[MOTOR_NUM],bool ENCODER_DISPLAY);
  void display_PID(MotorController cugo_motor_controllers[MOTOR_NUM],bool PID_CONTROLL_DISPLAY);
  int split(String data, char delimiter, String *dst);
  void cugo_motor_direct_instructions(int left, int right,MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cugo_rcmode(volatile unsigned long cugoRcTime[PWM_IN_MAX],MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cugo_stop_motor_immediately(MotorController cugo_motor_controllers[MOTOR_NUM]);
  void set_wait_time_cmd();
  void wait_time(int milisec);
  void matsu(int milisec);
  void matu(int milisec);
  void reset_pid_gain(MotorController cugo_motor_controllers[MOTOR_NUM]);
  void set_button_cmd();
  void go_backward(float distance,float max_velocity);
  void sagaru(float distance);
  void sagaru(float distance,float max_velocity);
  void turn_clockwise(float degree,float max_velocity);
  void migimawari(float degree);
  void migimawari(float degree,float max_velocity);
  void migimawari90();
  void migimawari90(float max_velocity);
  void migimawari45();
  void migimawari45(float max_velocity);
  void migimawari180();
  void migimawari180(float max_velocity);
  void go_forward(float distance,float max_velocity);
  void susumu(float distance);
  void susumu(float distance,float max_velocity);
  void turn_counter_clockwise(float degree,float max_velocity);
  void hidarimawari(float degree);
  void hidarimawari(float degree,float max_velocity);
  void hidarimawari90();
  void hidarimawari90(float max_velocity);
  void hidarimawari45();
  void hidarimawari45(float max_velocity);
  void hidarimawari180();
  void hidarimawari180(float max_velocity);
  void reset_arduino_mode_flags();
  void set_go_forward_cmd(MotorController cugo_motor_controllers[MOTOR_NUM]);
  void view_flags();
  void check_achievement_spi_cmd();
  void cmd_end(MotorController cugo_motor_controllers[MOTOR_NUM]);
  void check_achievement_wait_time_cmd(MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cmd_manager_flags_init(MotorController cugo_motor_controllers[MOTOR_NUM]);  
  void check_achievement_go_forward_cmd(MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cmd_manager(MotorController cugo_motor_controllers[MOTOR_NUM]);
  void check_achievement_button_cmd(MotorController cugo_motor_controllers[MOTOR_NUM]);
  void job_100ms(MotorController cugo_motor_controllers[MOTOR_NUM]);
  void job_1000ms();
  void display_detail(MotorController cugo_motor_controllers[MOTOR_NUM]);

/*-----------------------------------------------*/
/*MiddleUser向け関数*/ //★motorclassのたんなるラップアップは不要
  //初期設定関数関連
  void cugo_init();
  void cugo_check_mode_change(MotorController cugo_motor_controllers[MOTOR_NUM]);
  void cugo_wait_ms(int wait_ms,MotorController cugo_motor_controllers[MOTOR_NUM]);
  //前進制御＆回転制御
  //目標距離に前進または後進　位置制御あり
  void cugo_go(float target_distance,MotorController cugo_motor_controllers[MOTOR_NUM]);
  //目標距離と上限速度に従い前進または後進　位置制御あり
  void cugo_go(float target_distance,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]);//単位はm,rpm
  //目標距離と上限速度に従い前進または後進　位置制御なし
  void cugo_go_direct(float target_distance,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]);//単位はm,rpm
  //目標角度に回転　位置制御あり
  void cugo_turn(float target_degree,MotorController cugo_motor_controllers[MOTOR_NUM]);
  //目標角度と上限速度に従い回転　位置制御あり
  void cugo_turn(float target_degree,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]);//単位はm,rpm
  //目標角度と上限速度に従い自動で回転　位置制御なし
  void cugo_turn_direct(float target_degree,float target_rpm,MotorController cugo_motor_controllers[MOTOR_NUM]);//単位はm,rpm
  //カウント数のチェック
  int cugo_check_count_achivement(MotorController cugo_motor_controllers[MOTOR_NUM]);

  //モーター制御
  int cugo_check_a_channel_value(); //現状のachの値取得
  int cugo_check_b_channel_value(); //現状のbchの値取得
  int cugo_check_c_channel_value(); //現状のcchの値取得
  
  bool cugo_check_button(); //現状の押されているか
  int cugo_check_button_times(); //現状の押された回数
  void cugo_reset_button_times(); //現状の押された回数
  int cugo_button_press_time(); //ボタンの押されている時間
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
