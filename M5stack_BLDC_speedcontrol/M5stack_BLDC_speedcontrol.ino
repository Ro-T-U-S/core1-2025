#include <M5Stack.h>

// ESC制御用のPWM出力ピン（M5StackのG26を使用）
const int pwmPin = 26;

// ESCのPWM信号の範囲（マイクロ秒単位）
const int pwmMin = 1000;  // 停止（1ms）
const int pwmMax = 2000;  // 最大速度（2ms）

// 速度を5段階に分ける
const int speedLevels = 5;
int speedIndex = 0; // 初期速度（停止）

void setup() {
    M5.begin();
    pinMode(pwmPin, OUTPUT);
    ledcSetup(0, 50, 16); // PWMチャンネル0, 周波数50Hz (20ms周期), 16bit解像度
    ledcAttachPin(pwmPin, 0);
    
    Serial.begin(115200);
    Serial.println("ESC Control Initialized");

    // LCDの初期設定
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(10, 40);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println("ESC Control");
    
    // ESCのアーム手順を実行
    escArm();

    // 初期速度を表示
    updateSpeed();
}

void loop() {
    M5.update();

    // ボタンAで速度アップ
    if (M5.BtnA.wasPressed()) {
        if (speedIndex < speedLevels - 1) {
            speedIndex++;
            updateSpeed();
        }
    }

    // ボタンBで停止
    if (M5.BtnB.wasPressed()) {
        speedIndex = 0;
        updateSpeed();
    }

    // ボタンCで速度ダウン
    if (M5.BtnC.wasPressed()) {
        if (speedIndex > 0) {
            speedIndex--;
            updateSpeed();
        }
    }

    delay(100);
}

// 速度を更新する関数
void updateSpeed() {
    int pwmValue = map(speedIndex, 0, speedLevels - 1, pwmMin, pwmMax);
    ledcWrite(0, pwmValue * 65536L / 20000); // マイクロ秒をPWM値に変換
    
    Serial.print("Speed Level: ");
    Serial.print(speedIndex);
    Serial.print(" PWM: ");
    Serial.println(pwmValue);

    // LCDに表示を更新
    M5.Lcd.fillRect(10, 80, 220, 40, BLACK); // 数値部分をクリア
    M5.Lcd.setCursor(10, 80);
    M5.Lcd.print("Speed: ");
    M5.Lcd.print(speedIndex);
    M5.Lcd.print(" / ");
    M5.Lcd.print(speedLevels - 1);
}

// ESCをアームする関数（最小PWMを送信してESCを起動）
void escArm() {
    Serial.println("Arming ESC...");
    ledcWrite(0, pwmMin * 65536L / 20000);
    delay(3000); // ESCのアームが完了するまで待機
    Serial.println("ESC Armed");
}
