#include <Arduino.h>
#include <mcp_can.h>
#include <ESP32Servo.h>
#include "cybergear_controller.hh"

// 関数の前方宣言
void init_can();
float hexToFloat(String hexValue);
int hexToInt(String hexValue);
void escArm();
void updateSpeed();


int RX_PIN = 16; // 受信機の信号ピン(受信機の TX に接続)
int TX_PIN = 17; // 受信機の信号ピン(受信機の RX に接続)


#define CAN_INT 32 // MCP2515 CAN モジュールのINTピンを32番ピンに接続
MCP_CAN CAN(27); // MCP2515 CAN モジュールのCSピンを27番ピンに接続

// CyberGear モーターを設定する
uint8_t MASTER_CAN_ID = 0x00;
uint8_t FL_CAN_ID = 0x7F; // モーターの CAN ID
uint8_t BL_CAN_ID = 0x7E; // モーターの CAN ID
uint8_t FR_CAN_ID = 0x7D; // モーターの CAN ID
uint8_t BR_CAN_ID = 0x7C; // モーターの CAN ID
std::vector<uint8_t> motor_ids = {FL_CAN_ID, BL_CAN_ID, FR_CAN_ID, BR_CAN_ID};
std::vector<float> motor_speeds = {0.0f, 0.0f, 0.0f, 0.0f};

// CyberGear コントローラを設定する
CybergearController controller = CybergearController(MASTER_CAN_ID);
// 輪のベース径と半径を設定する
float wheel_base = 0.31f;
float wheel_radius = 0.05f;

// サーボモータの設定
int SERVO_PIN_R = 5;  // サーボモータの信号ピン
int SERVO_PIN_L = 14;  // サーボモータの信号ピン
Servo servo_r;  // サーボモータのインスタンスを作成
Servo servo_l;  // サーボモータのインスタンスを作成
int shoot_r = 0;  // サーボモータの初期位置(ゼロ度合わせ必要あり)
int shoot_l = 0;  // サーボモータの初期位置(ゼロ度合わせ必要あり)

// 射出モータ設定
Servo esc_l;
Servo esc_r;
const int injection_motor_r_pwm_pin = 21; // 射出モータpwm : injection_motor_pwm
const int injection_motor_l_pwm_pin = 22; // 射出モータpwm : injection_motor_pwm
// ESCのPWM信号の範囲（マイクロ秒単位）
const int pwmMin = 1000;  // 停止（1ms）
const int pwmMax = 2000;  // 最大速度（2ms）
// 速度を5段階に分ける
const int speedLevels = 5;
int speedIndex = 0; // 初期速度（停止）



void setup()
{
  // ESP32 のシリアル通信を開始する
  Serial.begin(115200);
  // 受信機のシリアル通信を開始する
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  // サーボモータの信号ピンを設定
  servo_r.attach(SERVO_PIN_R);
  servo_l.attach(SERVO_PIN_L);

  // CyberGear モーターの初期化
  init_can();
  Serial.println("Initializing CyberGear driver ...");
  controller.init(motor_ids, MODE_SPEED, &CAN);
  controller.set_speed_limit(motor_ids[0], 2.0f); // モーターの速度制限を設定する
  controller.set_speed_limit(motor_ids[1], 2.0f); // モーターの速度制限を設定する
  controller.set_speed_limit(motor_ids[2], 2.0f); // モーターの速度制限を設定する
  controller.set_speed_limit(motor_ids[3], 2.0f); // モーターの速度制限を設定する
  controller.enable_motors(); // モーターへの電力供給を開始する

  // 射出モータの初期化
  esc_r.attach(injection_motor_r_pwm_pin);
  esc_l.attach(injection_motor_l_pwm_pin);

  // ESCのアーム手順を実行
  escArm();
  // 初期速度を表示
  updateSpeed();
}


void loop()
{
  // 受信機からデータを読み取れる場合
  if (Serial1.available()) {
    // データを読み込み，パースする
    String message = Serial1.readStringUntil('\r\n'); // 改行までのデータを読み込む（送受信機のプロトコル依存）
    int index = message.indexOf("00,4701,");
    // 正常データの場合（例：00,0201,E0:01,50,50,50,50,00,00,00）
    if (index != -1 && index + 34 <= message.length()) {
      message = message.substring(index + 14, index + 34);
      // 緊急停止でない場合
      if (message.substring(0, 2) == "00") {
        // データからコマンドを抽出する
        float linear_x = hexToFloat(message.substring(3, 5));
        float linear_y = hexToFloat(message.substring(6, 8));
        float angular_z = hexToFloat(message.substring(9, 11));
        int shoot_r = hexToInt(message.substring(12, 14));
        int shoot_l = hexToInt(message.substring(15, 17));
        int triangle = hexToInt(message.substring(18, 20));
        
        // デバッグ用
        Serial.println(String(linear_x) + "," + String(linear_y) + "," + String(angular_z) + "," + String(shoot_r) + "," + String(shoot_l) + "," + String(triangle));
        
        // CyberGear モーターの速度を算出する（メカナムホイール用）
        float fl_speed = (linear_x - linear_y - angular_z * wheel_base) / wheel_radius;
        float fr_speed = -(linear_x + linear_y + angular_z * wheel_base) / wheel_radius; // モータの設置位置によって正負が異なる
        float bl_speed = (linear_x + linear_y - angular_z * wheel_base) / wheel_radius;
        float br_speed = -(linear_x - linear_y + angular_z * wheel_base) / wheel_radius; // モータの設置位置によって正負が異なる
        motor_speeds = {fl_speed, bl_speed, fr_speed, br_speed};
        
        if (shoot_r == 1)
        {
          Serial.println("Right side shoot");
          // servo_r.write(90);
          // delay(100);
          // servo_r.write(180);
          // delay(100);
          // servo_r.write(0);
          delay(100);
        }

        if (shoot_l == 1)
        {
          Serial.println("left side shoot");
          // servo_l.write(90);
          // delay(100);
          // servo_l.write(180);
          // delay(100);
          // servo_l.write(0);
          delay(100);
        }

        if (triangle == 1)
        {
          // 速度アップ
          // if (speedIndex < speedLevels - 1) {
          //   speedIndex++;
          //   updateSpeed();
          }
          // if (speedIndex > 0) {
          //   speedIndex--;
          //   updateSpeed();
          // }
        }
      }
    }
    // 受信機からデータを読み込めない場合
    else 
    {
      motor_speeds = {0.0, 0.0, 0.0, 0.0};
    }
    // CyberGear モーターにコマンドを転送する
    controller.send_speed_command(motor_ids, motor_speeds);
  

  // モーターのデータを更新・取得する
  std::vector<MotorStatus> status_list;
  if ( controller.process_can_packet() ) {
    controller.get_motor_status(status_list);
    // モーターの温度が0以下の場合
    if (status_list[0].temperature <= 0) {
      // 再度初期化処理を実行
      init_can();
      Serial.println("Re-initializing CyberGear driver ...");
      controller.init(motor_ids, MODE_SPEED, &CAN);
      controller.set_speed_limit(motor_ids[0], 2.0f); // モーターの速度制限を設定する
      controller.set_speed_limit(motor_ids[1], 2.0f); // モーターの速度制限を設定する
      controller.set_speed_limit(motor_ids[2], 2.0f); // モーターの速度制限を設定する
      controller.set_speed_limit(motor_ids[3], 2.0f); // モーターの速度制限を設定する
      controller.enable_motors(); // モーターへの電力供給を開始する
    }
  }
}


void init_can()
{
  Serial.println("Initializing CAN communication ...");
  CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);
}

float hexToFloat(String hexValue) {
  // 16 進数から整数に変換する
  int intValue = (int)strtol(hexValue.c_str(), NULL, 16);
  // 値範囲を 0 ~ 20 から -1.0 ~ 1.0 に変更する
  float floatValue = map(intValue, 0, 20, -10, 10) / 10.0;
  return floatValue;
}

int hexToInt(String hexValue) {
  // 16 進数から整数に変換する
  int intValue = (int)strtol(hexValue.c_str(), NULL, 16);
  return intValue;
}

// 速度を更新する関数
void updateSpeed() {
  int pwmValue = map(speedIndex, 0, speedLevels - 1, pwmMin, pwmMax);
  esc_r.writeMicroseconds(pwmValue * 65536L / 20000);
  esc_l.writeMicroseconds(pwmValue * 65536L / 20000);
    
  Serial.print("Speed Level: ");
  Serial.print(speedIndex);
  Serial.print(" PWM: ");
  Serial.println(pwmValue);
}

// ESCをアームする関数（最小PWMを送信してESCを起動）
void escArm() {
    Serial.println("Arming ESC...");
    esc_r.writeMicroseconds(pwmMin * 65536L / 20000);
    esc_l.writeMicroseconds(pwmMin * 65536L / 20000);
    delay(2000); // ESCのアームが完了するまで待機
    Serial.println("ESC Armed");
}