#include <Arduino.h>
#include <PS4Controller.h>

void setup();
void loop();

int RX_PIN = 16;
int TX_PIN = 17;
const char *bt_mac = "C8:2E:18:F0:07:36";

String cmd_data = "01,00,00,00,00,00,00";
int force_stop = 0;
int shoot_r = 0;
int shoot_l = 0;
int triangle = 0;
int cross = 0;
int circle = 0;

float normalizeData(float data, float old_initial, float old_final, float new_initial, float new_final) {
  // データを古い範囲から新しい範囲にマップする
  float normalizedValue = map(data, old_initial, old_final, new_initial, new_final);
  // 0.1 刻みになるよう変換する
  float scaledValue = normalizedValue / 10.0;

  return scaledValue;
}

String floatToHexStr(float value) {
  // デバッグ用
  if(value >= -0.2 && value <= 0.2) {
    value = 0.0;
  }

  // 値範囲を -1.0 ~ 1.0 から 0 ~ 20 に変更する
  int scaledValue = (value + 1.0) * 10;
  // scaledValue = constrain(scaledValue, 5, 15);

  // 整数を 16 進数に変換する
  char hexStr[3];
  sprintf(hexStr, "%02X", scaledValue);
  
  return String(hexStr);
}


void setup()
{
  // ESP32 のシリアル通信を開始する
  Serial.begin(115200);
  // 送信機のシリアル通信を開始する
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  // PS4 コントローラーに接続する
  PS4.begin(bt_mac);
  Serial.println("PS4 Con is Ready...");

  // 送信機が正常に起動するまでに待つ
  while(!(PS4.isConnected())){
    Serial.println("PS4 Controller is not conected yet. Wating......");
    delay(1000);
  }
}

void loop() 
{
  int LStickX = 0;
  int LStickY = 0;
  int RStickX = 0;
  int RStickY = 0;
  int L2Value = 0;
  int R2Value = 0;
  int Options = 0;
  int R1Value = 0;
  int L1Value = 0;
  int Triangle = 0;
  int Cross = 0;
  int Circle =0;

  if (PS4.isConnected()) {
    // コントローラー値を取得する
    LStickX = PS4.LStickX();
    LStickY = PS4.LStickY();
    RStickX = PS4.RStickX();
    RStickY = PS4.RStickY();
    L2Value = PS4.L2Value();
    R2Value = PS4.R2Value();
    Options = PS4.Options();
    R1Value = PS4.R1();
    L1Value = PS4.L1();
    Triangle = PS4.Triangle();
    Cross = PS4.Cross();
    Circle = PS4.Circle();

    // 緊急停止
    if (Options && !force_stop) {
      Serial.println("Stop operation ...");
      force_stop = 1;
    }
    else if (Options && force_stop) {
      Serial.println("Resume operation ...");
      force_stop = 0;
    }

    // 射出 R
    if (R1Value > 0) {
      Serial.println("Shoot R disk ...");
      shoot_r = 1;
    }
    else {
      shoot_r = 0;
    }
    
    // 射出
    if (L1Value > 0) {
      Serial.println("Shoot L disk ...");
      shoot_l = 1;
    }
    else {
      shoot_l = 0;
    }

    if (Triangle > 0)
    {
      triangle = 1;
    }
    else
    {
      triangle =0;
    }

    if (Cross > 0)
    {
      cross = 1;
    }
    else
    {
      cross = 0;
    }

    if (Circle > 0)
    {
      circle = 1;
    }
    else
    {
      circle = 0;
    }


    // 線速度と角速度に変換する
    float linear_x = normalizeData(LStickY, -128, 127, -10.0, 10.0);
    float linear_y = -normalizeData(LStickX, -128, 127, -10.0, 10.0);
    float angular_z = normalizeData(RStickX, -128, 127, -10.0, 10.0);

    // R1 または L1 が押されている場合は、横方向移動を変更する
    if (R2Value > 0) {
      linear_y = -normalizeData(R2Value, 0, 255, 0.0, 10.0);
    } else if (L2Value > 0) {
      linear_y = normalizeData(L2Value, 0, 255, 0.0, 10.0);
    }
    // コマンドデータ
    cmd_data = "0" + String(force_stop) + "," + floatToHexStr(linear_x) + "," + floatToHexStr(linear_y) + "," 
                   + floatToHexStr(angular_z) + ",0" + String(shoot_r) + ",0" + String(shoot_l) + ",0" + String(triangle);
    
    // デバッグ用（送信するデータの確認）
    // Serial.println("00," + String(linear_x) + "," + String(linear_y) + "," + String(angular_z) + ",00,00,00");
  }

  // 送信機にデータを転送する（送信間隔 150ms）
  Serial2.println(cmd_data);
  delay(150);

  // デバッグ用（送信できるかどうか）
  Serial.println(cmd_data);
  String transmitter_status = Serial2.readStringUntil('\r\n');
  Serial.println(transmitter_status);
}