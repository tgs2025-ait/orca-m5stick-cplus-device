#include <M5Unified.h>
#include <esp32-hal-cpu.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
float AX, AY, AZ;

// 閾値設定
const float TARGET_X   = 0.15f;
const float TARGET_Y   = 0.20f;
const float TARGET_Z   = 1.00f;
const float ADJUSTMENT = 1.00f;

// ORCA TRUE 検出後のホールド時間（ms）
const unsigned long HOLD_TIME = 1000;

// 最後に TRUE 検出を行ったタイムスタンプ
unsigned long lastDetectTime = 0;

void setup() {
  M5.begin();
  setCpuFrequencyMhz(80);
  M5.Lcd.setBrightness(100);
  M5.Imu.init();                     // IMU 初期化  [oai_citation:0‡M5Stack Docs](https://docs.m5stack.com/en/arduino/m5unified/imu_class?utm_source=chatgpt.com)

  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextFont(2);
  M5.Lcd.setTextSize(3);

  Serial.begin(115200);
  SerialBT.begin("orca-m5stick-c-plus-device");
  delay(100);
}

void loop() {
  M5.update();
  unsigned long now = millis();

  // ホールド期間中は判定をスキップ
  if (now - lastDetectTime >= HOLD_TIME) {
    M5.Imu.getAccel(&AX, &AY, &AZ);

    bool xTrue = fabs(AX - TARGET_X) >= ADJUSTMENT;
    bool yTrue = fabs(AY - TARGET_Y) >= ADJUSTMENT;
    bool zTrue = fabs(AZ - TARGET_Z) >= ADJUSTMENT;

    // いずれかが成り立ったら ORCA TRUE
    if (xTrue || yTrue || zTrue) {
      Serial.println("ORCA TRUE");
      SerialBT.println("ORCA TRUE");
      lastDetectTime = now;  // ホールド開始
    }
  }

  // LCD 表示（常に更新）
  int batLevel = M5.Power.getBatteryLevel();
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("Bat:%3d%%\n", batLevel);
  M5.Lcd.printf("X:%6.2f\nY:%6.2f\nZ:%6.2f\n", AX, AY, AZ);

  // シリアルプロッタ用出力
  Serial.printf("Bat:%d%% AX:%.2f AY:%.2f AZ:%.2f\n", batLevel, AX, AY, AZ);
  SerialBT.printf("Bat:%d%% AX:%.2f AY:%.2f AZ:%.2f\n", batLevel, AX, AY, AZ);

  delay(200);
}