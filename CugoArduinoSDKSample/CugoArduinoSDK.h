// CugoArduinoMiddleUserProgramming用ライブラリ
  #ifndef CUGOARDUINOMODE_H
  #define CUGOARDUINOMODE_H

//MotorControllerクラスの利用
  #include "MotorController.h"

// モータとエンコーダのピン配置設定
  #define CUGO_PIN_MOTOR_L A0  // モータ出力ピン(L)
  #define CUGO_PIN_MOTOR_R A1  // モータ出力ピン(R)
  #define CUGO_PIN_ENCODER_L_A 2  // エンコーダ割り込み入力ピン(L)
  #define CUGO_PIN_ENCODER_L_B 8  // エンコーダ回転方向入力ピン(L)
  #define CUGO_PIN_ENCODER_R_A 3  // エンコーダ割り込み入力ピン(R)
  #define CUGO_PIN_ENCODER_R_B 9  // エンコーダ回転方向入力ピン(R)

// プロポ信号の読み取りピン（L/R/MODE_CAHNGE）
  #define CUGO_PWM_IN_PIN0   5   // プロポスティック入力ピン(L)//digitalRead ピン変化割り込みの設定
  #define CUGO_PWM_IN_PIN1   6   // プロポスティック入力ピン(MODE)//digitalRead ピン変化割り込みの設定
  #define CUGO_PWM_IN_PIN2   7   // プロポスティック入力ピン(R)//digitalRead ピン変化割り込みの設定

//cugo仕様関連
  #define CUGO_WHEEL_RADIUS_L  0.03858d
  #define CUGO_WHEEL_RADIUS_R  0.03858d
  #define CUGO_TREAD  0.380d
  #define CUGO_ENCODER_RESOLUTION  2048.0d 
  #define CUGO_MAX_MOTOR_RPM 180 //モータの速度上限値
  //上記の仕様が変更される場合は下の変換係数も変更してください。
    #define CUGO_CONVERSION_DISTANCE_TO_COUNT 8448.660535308492d // 変換係数： CUGO_ENCODER_RESOLUTION / (2 * CUGO_WHEEL_RADIUS_L * PI)の計算結果
    #define CUGO_CONVERSION_COUNT_TO_DISTANCE 0.000118361958d    // 変換係数： 2 * CUGO_WHEEL_RADIUS_L * PI  / CUGO_ENCODER_RESOLUTION の計算結果

// PID速度制御ゲイン調整　MotorControllerで使用
  #define CUGO_L_KP   1.0f   //CuGoV3
  #define CUGO_L_KI   0.02f   //CuGoV3
  #define CUGO_L_KD   0.1f   //CuGoV3
  //const float CUGO_L_KP = 1.0;
  //const float CUGO_L_KI = 0.06;
  //const float CUGO_L_KD = 0.1;
  #define CUGO_R_KP   1.0f   //CuGoV3
  #define CUGO_R_KI   0.02f   //CuGoV3
  #define CUGO_R_KD   0.1f   //CuGoV3
  //const float CUGO_R_KP = 1.0;
  //const float CUGO_R_KI = 0.06;
  //const float CUGO_R_KD = 0.1;

  // ローパスフィルタ
    #define CUGO_L_LPF   0.2f 
    #define CUGO_R_LPF   0.2f
    //const float CUGO_L_LPF = 0.2;
    //const float CUGO_R_LPF = 0.2;
    #define CUGO_L_MAX_COUNT_I  9000.0f //速度上限を設定している場合はiは必ず0に
    #define CUGO_R_MAX_COUNT_I  9000.0f //速度上限を設定している場合はiは必ず0に

// PID位置制御のゲイン調整
  #define CUGO_L_COUNT_KP  50.0f
  #define CUGO_L_COUNT_KI  0.5f 
  #define CUGO_L_COUNT_KD  10.0f
  #define CUGO_R_COUNT_KP  50.0f
  #define CUGO_R_COUNT_KI  0.5f 
  #define CUGO_R_COUNT_KD  10.0f
  #define CUGO_L_MAX_COUNT_I  120 
  #define CUGO_R_MAX_COUNT_I  120 

// CugoArduinoキットのスタートボタン
  #define CUGO_CMD_BUTTON_PIN A2 

// 各種動作モード定義
  #define CUGO_RC_MODE 0
  #define CUGO_ARDUINO_MODE 1

  //プロポ設定
    #define CUGO_PROPO_A 0
    #define CUGO_PROPO_B 1
    #define CUGO_PROPO_C 2

  //オドメトリ設定
    #define CUGO_ODO_X 0
    #define CUGO_ODO_Y 1
    #define CUGO_ODO_THETA 2
    #define CUGO_ODO_DEGREE 3

  //モーター設定
    #define CUGO_MOTOR_NUM 2 // モータ接続数
    #define CUGO_MOTOR_LEFT 0
    #define CUGO_MOTOR_RIGHT 1

//各種閾値
  #define CUGO_PROPO_MAX   1900  // ARDUINOモードに入るときの閾値(us) (1100~1900/中央1500)
  #define CUGO_PROPO_MIN   1100  // ARDUINOモードに入るときの閾値(us) (1100~1900/中央1500)
  #define CUGO_ARDUINO_MODE_IN   1700  // ARDUINOモードに入るときの閾値(us) (1100~1900/中央1500)
  #define CUGO_ARDUINO_MODE_OUT  1300  // ARDUINOモードから抜けるときの閾値(us) (1100~1900/中央1500)
  #define CUGO_EXCEPTION_NO -32768 //int下限
  #define CUGO_NORMAL_MOTOR_RPM 50
  #define CUGO_BUTTON_CHECK_BORDER 50000

//割り込みPIN関連 
  #define CUGO_PIN_UP(no)    cugoUpTime[no] = micros();
  #define CUGO_PIN_DOWN(no)  cugo_time[no] = micros() - cugoUpTime[no]
  #define CUGO_PWM_IN_MAX  4

// グローバル変数宣言※関数はCugoArduinoMiddle.cppに記載
  extern int cugo_old_runmode;
  extern int cugo_button_count;
  extern long int cugo_count_prev_L;
  extern long int cugo_count_prev_R;
  extern unsigned long long int calc_odometer_time;
  extern float cugo_odometer_theta;
  extern float cugo_odometer_x;
  extern float cugo_odometer_y;
  extern float cugo_odometer_degree;
  extern long int cugo_target_count_L;
  extern long int cugo_target_count_R;
  extern long int cugo_odometer_count_theta;
  extern int cugoRunMode;
  extern const bool CUGO_L_reverse;
  extern const bool CUGO_R_reverse;
  extern bool cugo_direction_L; 
  extern bool cugo_direction_R; 
  extern bool cugo_button_check;
  extern bool cugo_button_check;
  extern int CUGO_OLD_CMD_BUTTON_VALUE; 
  extern int CUGO_OLD_PWM_IN_PIN0_VALUE; 
  extern int CUGO_OLD_PWM_IN_PIN1_VALUE; 
  extern int CUGO_OLD_PWM_IN_PIN2_VALUE; 
  extern volatile unsigned long cugoUpTime[CUGO_PWM_IN_MAX];
  extern volatile unsigned long cugoRcTime[CUGO_PWM_IN_MAX];
  extern volatile unsigned long cugo_time[CUGO_PWM_IN_MAX];
  extern volatile unsigned long long cugoButtonTime;

//各種関数
  //初期設定関数関連
    void cugo_init();
    void cugo_init_display();
    void cugo_init_KOPROPO(int CUGO_OLD_PWM_IN_PIN0_VALUE,int CUGO_OLD_PWM_IN_PIN1_VALUE,int CUGO_OLD_PWM_IN_PIN2_VALUE);
    void cugo_reset_pid_gain(MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);
    void cugo_check_mode_change(MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);

  //モータ直接制御関連
    void cugo_motor_direct_instructions(int left, int right,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);
    void cugo_rcmode(volatile unsigned long cugoRcTime[CUGO_PWM_IN_MAX],MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);
    void cugo_stop(MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);

  //前進後進、回転、円軌道関数
    void cugo_move_forward(float target_distance,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);
    void cugo_move_forward(float target_distance,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);//単位はm,rpm
    void cugo_move_forward_raw(float target_distance,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);//単位はm,rpm
    void cugo_turn_clockwise(float target_degree,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);
    void cugo_turn_clockwise(float target_degree,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);//単位はm,rpm
    void cugo_turn_clockwise_raw(float target_degree,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);//単位はm,rpm
    void cugo_turn_counterclockwise(float target_degree,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);
    void cugo_turn_counterclockwise(float target_degree,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);//単位はm,rpm
    void cugo_turn_counterclockwise_raw(float target_degree,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);//単位はm,rpm  

  //極座標での移動命令関数
    void cugo_curve_theta_raw(float target_radius,float target_theta,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);
    void cugo_curve_distance_raw(float target_radius,float target_distance,float target_rpm,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);

  //wait関数
    void cugo_wait(unsigned long long int  wait_ms);
    void cugo_long_wait(unsigned long long int wait_seconds);

  //プロポ入力確認関数
    int cugo_check_propo_channel_value(int channel_number); 
  //ボタン関連の関数
    bool cugo_check_button(); //現状の押されているか
    int cugo_check_button_times(); //現状の押された回数
    void cugo_reset_button_times(); //現状の押された回数の初期化
    long int cugo_button_press_time(); //ボタンの押されている時間

  //オドメトリ関連関数
    float cugo_check_odometer(int check_number); 
    void cugo_start_odometer();
    void cugo_calc_odometer(MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);
    void cugo_reset_odometer();
  //その他関数
    void cugo_calc_necessary_rotate(float degree,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]); 
    void cugo_calc_necessary_count(float distance,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]); 
    bool cugo_check_count_achievement(int motor_num_,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);
    void cugo_move_pid(float target_rpm,bool use_pid,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);//単位はm,rpm
  //テスト関数
    void cugo_test(int test_number,MotorController cugo_motor_controllers[CUGO_MOTOR_NUM]);//テスト用関数

#endif
