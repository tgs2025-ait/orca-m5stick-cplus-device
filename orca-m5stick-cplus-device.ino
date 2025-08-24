/*
 * M5StickC Plus（初代, MPU6886=6軸） + M5Unified 版
 * 目的：
 *  - デバッグ時:
 *      LCD → バッテリー + 加速度
 *      BT  → シリアルプロッタ形式（ax ay az）
 *  - 本番時:
 *      LCD → バッテリーのみ
 *      BT  → "ax:{}, ay:{}, az:{}" のラベル付き形式
 *  - loop() 内で 100 ms 周期
 */

#include <M5Unified.h>
#include <esp32-hal-cpu.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// --- IMUデータ ---
float accX=0, accY=0, accZ=0;  // [g]

// --- 周期制御 ---
static uint32_t next_ms = 0;   // 100msの基準時刻

// --- 動作モード切り替え ---
// true = デバッグモード（LCDに加速度表示 & BTはプロッタ形式）
// false = 本番モード（LCDはバッテリーのみ & BTはラベル付き出力）
bool debugMode = false;   // ← ここで切り替え

//==================== プロトタイプ ====================//
void initM5();
void initBluetooth();
void renderUI(int batteryPct, float ax, float ay, float az);
void outputBT(float ax, float ay, float az);

//==================== Arduino標準エントリ ====================//
void setup() {
  initM5();
  initBluetooth();
  next_ms = millis() + 100;   // 最初の基準
}

void loop() {
  M5.update();

  // 1) IMU生データ取得
  M5.Imu.getAccel(&accX, &accY, &accZ);

  // 2) バッテリー残量[%]
  int batPct = M5.Power.getBatteryLevel();

  // 3) 表示・出力
  renderUI(batPct, accX, accY, accZ);
  outputBT(accX, accY, accZ);

  // 4) 周期100ms維持
  uint32_t now = millis();
  if (now < next_ms) {
    delay(next_ms - now);
  }
  next_ms += 100;
  if ((int32_t)(millis() - next_ms) > 0) {
    next_ms = millis() + 100;
  }
}

//==================== 関数本体 ====================//
void initM5() {
  auto cfg = M5.config();
  cfg.internal_imu = true;
  cfg.clear_display = true;
  M5.begin(cfg);

  setCpuFrequencyMhz(80);
  M5.Display.setBrightness(128);
  M5.Display.setRotation(3);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
}

void initBluetooth() {
  SerialBT.begin("orca-m5stick-c-plus-device");
  delay(100);
}

// LCD表示
void renderUI(int batteryPct, float ax, float ay, float az) {
  M5.Display.setCursor(0, 0);
  M5.Display.printf("Bat:%3d%%\n", batteryPct);

  if (debugMode) {
    // デバッグモード時のみ加速度表示
    M5.Display.setCursor(0, 30);
    M5.Display.printf("aX:%6.2f aY:%6.2f aZ:%6.2f\n", ax, ay, az);
  }
}

// Bluetooth出力
void outputBT(float ax, float ay, float az) {
  if (debugMode) {
    // Arduino IDE シリアルプロッタ形式（数値のみ 空白区切り）
    SerialBT.printf("%.3f %.3f %.3f\n", ax, ay, az);
  } else {
    // 本番モード：機械可読なラベル付き形式（1行=1サンプル）
    // 例: ax:0.123, ay:-0.045, az:1.002
    SerialBT.printf("ax:%.3f, ay:%.3f, az:%.3f\n", ax, ay, az);
  }
}