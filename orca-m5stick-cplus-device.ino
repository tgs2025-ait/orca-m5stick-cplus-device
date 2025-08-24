/*
 * M5StickC Plus（初代, MPU6886=6軸） + M5Unified 版
 * 目的：
 *  - デバッグ時 (debugMode = true):
 *      LCD → バッテリー + 加速度を100msごとに更新
 *      BT  → シリアルプロッタ形式（空白区切り数値）
 *  - 本番時 (debugMode = false):
 *      LCD → バッテリーのみ表示。画面は出しっぱなしで 60秒ごとにだけ更新
 *      BT  → "ax:..., ay:..., az:..." ラベル付き形式を100msごとに送信
 */

#include <M5Unified.h>
#include <esp32-hal-cpu.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// --- IMUデータ ---
float accX=0, accY=0, accZ=0;  // [g]

// --- 周期制御 ---
static uint32_t next_ms = 0;          // 100msごと
static uint32_t next_display_ms = 0;  // LCD更新タイマー（本番時 60sごと）

// --- モード切替 ---
bool debugMode = false;  // ← 本番: false / デバッグ: true

// プロトタイプ
void initM5();
void initBluetooth();
void renderUI(int batteryPct, float ax, float ay, float az, bool showAccel);
void outputBT(float ax, float ay, float az);

void setup() {
  initM5();
  initBluetooth();
  next_ms = millis() + 100;

  // 起動直後に一度表示（本番でもすぐに電池%が出る）
  int batPct = M5.Power.getBatteryLevel();
  M5.Imu.getAccel(&accX, &accY, &accZ);
  renderUI(batPct, accX, accY, accZ, debugMode);  // 本番時は電池のみ表示
  next_display_ms = millis() + (debugMode ? 100 : 60000);
}

void loop() {
  M5.update();

  // 1) IMU
  M5.Imu.getAccel(&accX, &accY, &accZ);

  // 2) バッテリー％（M5Unified提供）
  int batPct = M5.Power.getBatteryLevel();

  // 3) LCD更新：デバッグは100msごと、本番は60秒ごと
  uint32_t now = millis();
  if (now >= next_display_ms) {
    renderUI(batPct, accX, accY, accZ, debugMode);  // 本番:電池のみ／デバッグ:電池+加速度
    next_display_ms = now + (debugMode ? 100 : 60000);
  }

  // 4) Bluetooth出力（常に100msごと）
  outputBT(accX, accY, accZ);

  // 5) 周期100ms維持（ドリフトしにくい加算方式＋補正）
  now = millis();
  if (now < next_ms) delay(next_ms - now);
  next_ms += 100;
  if ((int32_t)(millis() - next_ms) > 0) next_ms = millis() + 100;
}

//==================== 関数本体 ====================//
void initM5() {
  auto cfg = M5.config();
  cfg.internal_imu = true;
  cfg.clear_display = true;
  M5.begin(cfg);

  setCpuFrequencyMhz(80);
  M5.Display.setBrightness(64);//MAX: 255
  M5.Display.setRotation(3);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
}

void initBluetooth() {
  SerialBT.begin("orca-m5stick-c-plus-device");
  delay(100);  // 初期化直後の安定待ち
}

// showAccel=true: 電池+加速度, false: 電池のみ
void renderUI(int batteryPct, float ax, float ay, float az, bool showAccel) {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.printf("Bat:%3d%%\n", batteryPct);
  if (showAccel) {
    M5.Display.setCursor(0, 30);
    M5.Display.printf("aX:%6.2f aY:%6.2f aZ:%6.2f\n", ax, ay, az);
  }
}

// デバッグ: プロッタ形式 / 本番: ラベル付き
void outputBT(float ax, float ay, float az) {
  if (debugMode) {
    SerialBT.printf("%.3f %.3f %.3f\n", ax, ay, az);
  } else {
    SerialBT.printf("ax:%.3f, ay:%.3f, az:%.3f\n", ax, ay, az);
  }
}