/*
 * M5StickC Plus（初代, MPU6886=6軸） + M5Unified + MadgwickAHRS
 * - キャリブレーションは BtnA（Aボタン）押下時のみ実行（起動時は実行しない）
 * - 未キャリブ状態でもバッテリー残量を表示し、「Press A and Calibrate」を促す
 * - USB給電状態（ON/OFF と V/I）を表示
 * - ジャイロ/加速度とも デフォルト1500 サンプル平均（約6〜7秒）
 * - 進捗表示（プログレスバー）
 * - 姿勢推定・区間5移動平均
 * - BT出力（ay/pitch/roll）
 * - USBシリアルプロッタ（ay/pitch/roll + Min/Max）
 * ★ Bluetooth接続状態をディスプレイに表示する機能を追加
 */

#include <M5Unified.h>
#include <esp32-hal-cpu.h>
#include "BluetoothSerial.h"
#include <MadgwickAHRS.h>
#include <math.h>

BluetoothSerial SerialBT;
Madgwick filter;

// --- IMUデータ（生） ---
float accX=0, accY=0, accZ=0;     // [g]
float gyroX=0, gyroY=0, gyroZ=0;  // [deg/s]

// --- オフセット ---
float gOffX=0.0f, gOffY=0.0f, gOffZ=0.0f;   // ジャイロ [deg/s]
float aOffX=0.0f, aOffY=0.0f, aOffZ=0.0f;   // 加速度 [g]

// --- 姿勢角（deg） ---
float roll_deg=0.0f, pitch_deg=0.0f, yaw_deg=0.0f;

// --- 移動平均（区間5） ---
static const int MA_N = 5;
float maRoll[MA_N] = {0}, maPitch[MA_N] = {0}, maYaw[MA_N] = {0};
float sumRoll=0, sumPitch=0, sumYaw=0;
int   maIdx=0, maCount=0;

// --- 周期制御 ---
static uint32_t next_ms = 0;          // 100msごと
static uint32_t next_display_ms = 0;  // LCD更新タイマー（本番時 60sごと）

// --- モード切替 ---
bool debugMode = false;  // ← 本番: false / デバッグ: true

// --- 精度優先キャリブ用サンプル数（必要なら変更） ---
static const uint16_t CALIB_G_SAMPLES = 1500;
static const uint16_t CALIB_A_SAMPLES = 1500;

// --- シリアルプロッタ設定（Y軸レンジの“実質固定”） ---
const bool   ENABLE_SERIAL_PLOTTER = true;   // 必要なければ false
const float  PLOT_YMIN = -180.0f;            // 例：姿勢角の下限
const float  PLOT_YMAX =  180.0f;            // 例：姿勢角の上限
bool         plotterHeaderSent = false;      // 凡例行を一度だけ送るため

// --- 起動後のキャリブ実施フラグ ---
bool calibratedOnce = false;

// --- USB検出用しきい値 ---
static const float VBUS_ON_THRESHOLD_V = 4.5f;

// プロトタイプ
void initM5();
void initBluetooth();
void calibrateGyroOffset(uint16_t samples);
void calibrateAccelOffset(uint16_t samples);
void doOnDemandCalibration(uint16_t gSamples, uint16_t aSamples);
void resetAverages();
void drawProgress(const char* title, uint16_t done, uint16_t total);
void renderUI(int batteryPct, float ax, float ay, float az,
              float roll_d, float pitch_d, float yaw_d, bool showDebug);
void outputBT(float ax, float ay, float az, float roll_d, float pitch_d, float yaw_d);
void outputSerialPlotter(float ax, float ay, float az, float roll_d, float pitch_d, float yaw_d);
void maPush(float r, float p, float y, float &r_avg, float &p_avg, float &y_avg);

void setup() {
  initM5();
  initBluetooth();

  // USBシリアル: プロッタ用
  Serial.begin(115200);
  delay(50);

  // Madgwick を 10Hz 前提で初期化（loopは100ms周期）
  filter.begin(10);

  // ★起動時キャリブレーションは実行しない★

  next_ms = millis() + 100;

  // 初回更新（現オフセット=0で一度回して初期姿勢をセット）
  int batPct = M5.Power.getBatteryLevel();
  M5.Imu.getAccel(&accX, &accY, &accZ);
  M5.Imu.getGyro(&gyroX, &gyroY, &gyroZ);

  float ax = accX - aOffX;
  float ay = accY - aOffY;
  float az = accZ - aOffZ;
  float gx = gyroX - gOffX;
  float gy = gyroY - gOffY;
  float gz = gyroZ - gOffZ;

  filter.updateIMU(gx, gy, gz, ax, ay, az);
  roll_deg  = filter.getRoll();
  pitch_deg = filter.getPitch();
  yaw_deg   = filter.getYaw();

  resetAverages();
  float rAvg, pAvg, yAvg;
  maPush(roll_deg, pitch_deg, yaw_deg, rAvg, pAvg, yAvg);

  renderUI(batPct, ax, ay, az, rAvg, pAvg, yAvg, debugMode);

  // シリアルプロッタ：凡例行
  if (ENABLE_SERIAL_PLOTTER && !plotterHeaderSent) {
    Serial.println("ay,pitch,roll,Min,Max");
    plotterHeaderSent = true;
  }

  next_display_ms = millis() + (debugMode ? 100 : 60000);
}

void loop() {
  M5.update();

  // BtnA（正面）でキャリブレーション（★ボタン押下時のみ実行★）
  if (M5.BtnA.wasPressed()) {
    doOnDemandCalibration(CALIB_G_SAMPLES, CALIB_A_SAMPLES);
  }

  // 1) IMU 読み出し
  M5.Imu.getAccel(&accX, &accY, &accZ);     // [g]
  M5.Imu.getGyro(&gyroX, &gyroY, &gyroZ);   // [deg/s]

  // 2) オフセット補正
  float ax = accX - aOffX;
  float ay = accY - aOffY;
  float az = accZ - aOffZ;
  float gx = gyroX - gOffX;
  float gy = gyroY - gOffY;
  float gz = gyroZ - gOffZ;

  // 3) Madgwick 更新（磁気なし=IMUモード）
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  // 4) 姿勢角（生）→ 移動平均
  float rRaw = filter.getRoll();
  float pRaw = filter.getPitch();
  float yRaw = filter.getYaw();   // 6軸のため長期ドリフトあり

  float rAvg, pAvg, yAvg;
  maPush(rRaw, pRaw, yRaw, rAvg, pAvg, yAvg);
  roll_deg  = rAvg;
  pitch_deg = pAvg;
  yaw_deg   = yAvg;

  // 5) バッテリー％
  int batPct = M5.Power.getBatteryLevel();

  // 6) LCD更新
  uint32_t now = millis();
  if (now >= next_display_ms) {
    renderUI(batPct, ax, ay, az, roll_deg, pitch_deg, yaw_deg, debugMode);
    next_display_ms = now + (debugMode ? 100 : 60000);
  }

  // 7) Bluetooth出力（ay + 姿勢角（平滑後））
  outputBT(ax, ay, az, roll_deg, pitch_deg, yaw_deg);

  // 8) シリアルプロッタ出力（固定線Min/Max付き）
  if (ENABLE_SERIAL_PLOTTER) {
    outputSerialPlotter(ax, ay, az, roll_deg, pitch_deg, yaw_deg);
  }

  // 9) 周期100ms維持
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
  M5.Display.setBrightness(64); // MAX: 255
  M5.Display.setRotation(3);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);

  M5.Imu.begin();  // 念のため明示初期化
}

void initBluetooth() {
  SerialBT.begin("orca-m5stick-c-plus-device");
  delay(100);
}

// ---- 進捗バー描画 ----
void drawProgress(const char* title, uint16_t done, uint16_t total) {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.println(title);
  float ratio = (total == 0) ? 0.f : (float)done / (float)total;
  if (ratio < 0) ratio = 0;
  if (ratio > 1) ratio = 1;
  int barW = M5.Display.width() - 16;
  int filled = (int)(barW * ratio);
  int x = 8, y = 40, h = 16;
  M5.Display.drawRect(x, y, barW, h, TFT_WHITE);
  M5.Display.fillRect(x, y, filled, h, TFT_WHITE);
  M5.Display.setCursor(0, 64);
  M5.Display.printf("%u / %u (%.0f%%)\n", done, total, ratio*100.f);
}

// ---- キャリブレーション ----
void calibrateGyroOffset(uint16_t samples) {
  float sx=0, sy=0, sz=0;
  for (uint16_t i=0; i<samples; ++i) {
    float gx, gy, gz;
    M5.Imu.getGyro(&gx, &gy, &gz);
    sx += gx; sy += gy; sz += gz;
    if ((i % 10) == 0) drawProgress("Calib Gyro: keep still", i, samples);
    delay(2);
  }
  gOffX = sx / samples;
  gOffY = sy / samples;
  gOffZ = sz / samples;
}

void calibrateAccelOffset(uint16_t samples) {
  // 水平・静止を想定: ax≈0g, ay≈0g, az≈+1g（表向き）
  const float EX_AX = 0.0f;
  const float EX_AY = 0.0f;
  const float EX_AZ = 1.0f;

  float sx=0, sy=0, sz=0;
  for (uint16_t i=0; i<samples; ++i) {
    float ax, ay, az;
    M5.Imu.getAccel(&ax, &ay, &az);
    sx += ax; sy += ay; sz += az;
    if ((i % 10) == 0) drawProgress("Calib Accel: keep flat", i, samples);
    delay(2);
  }
  float axm = sx / samples;
  float aym = sy / samples;
  float azm = sz / samples;

  aOffX = axm - EX_AX;
  aOffY = aym - EX_AY;
  aOffZ = azm - EX_AZ;
}

// ---- On-demand キャリブレーション（BtnA押下時に呼ぶ）----
void doOnDemandCalibration(uint16_t gSamples, uint16_t aSamples) {
  // 案内
  drawProgress("Prepare: lay flat & still", 0, 100);
  delay(500);

  calibrateGyroOffset(gSamples);
  calibrateAccelOffset(aSamples);

  // フィルタ/移動平均をリセット
  filter.begin(10);
  resetAverages();

  // 初期一回更新で姿勢をセット
  M5.Imu.getAccel(&accX, &accY, &accZ);
  M5.Imu.getGyro(&gyroX, &gyroY, &gyroZ);

  float ax = accX - aOffX;
  float ay = accY - aOffY;
  float az = accZ - aOffZ;
  float gx = gyroX - gOffX;
  float gy = gyroY - gOffY;
  float gz = gyroZ - gOffZ;
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  float r = filter.getRoll();
  float p = filter.getPitch();
  float y = filter.getYaw();
  float rAvg, pAvg, yAvg;
  maPush(r, p, y, rAvg, pAvg, yAvg);

  // 完了表示
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.println("Calibration done");
  M5.Display.printf("aOfs:%+.3f %+.3f %+.3f\n", aOffX, aOffY, aOffZ);
  M5.Display.printf("gOfs:%+.3f %+.3f %+.3f\n", gOffX, gOffY, gOffZ);
  delay(800);

  if (SerialBT.hasClient()) { // ★変更点：接続中のみ送信
    SerialBT.printf("CALIB_DONE, aOff=(%.4f,%.4f,%.4f), gOff=(%.4f,%.4f,%.4f)\n",
                    aOffX, aOffY, aOffZ, gOffX, gOffY, gOffZ);
  }
  
  // ★ キャリブ済みフラグを立てる
  calibratedOnce = true;
}

// ---- 移動平均の初期化 ----
void resetAverages() {
  for (int i=0;i<MA_N;++i){ maRoll[i]=maPitch[i]=maYaw[i]=0.0f; }
  sumRoll=sumPitch=sumYaw=0.0f;
  maIdx=0;
  maCount=0;
}

// ---- 表示 ----
// ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
// ★ 変更点：この関数にBluetooth接続状態の表示を追加 ★
// ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
void renderUI(int batteryPct, float ax, float ay, float az,
              float roll_d, float pitch_d, float yaw_d, bool showDebug) {
  // 取得：USB VBUSの電圧/電流（M5Unified: AXP192）
  float vbus = M5.Power.Axp192.getVBUSVoltage(); // [V]
  float ibus = M5.Power.Axp192.getVBUSCurrent(); // [mA]
  bool usb_on = (vbus > VBUS_ON_THRESHOLD_V);

  M5.Display.fillScreen(TFT_BLACK);
  
  // 1行目にバッテリーとBT状態
  M5.Display.setCursor(0, 0);
  M5.Display.printf("Bat:%3d%%", batteryPct); // 改行なしに変更

  // BT状態を右側に表示
  M5.Display.setCursor(130, 0); // 表示位置を調整
  if (SerialBT.hasClient()) {
    M5.Display.setTextColor(TFT_GREEN); // 接続中は緑
    M5.Display.print("BT:OK");
  } else {
    M5.Display.setTextColor(TFT_WHITE); // 未接続は白
    M5.Display.print("BT:Wait");
  }
  M5.Display.setTextColor(TFT_WHITE); // 文字色を白に戻す
  M5.Display.println(); // ここで改行

  // 2行目にUSB状態
  M5.Display.setCursor(0, 18);
  if (usb_on) {
    M5.Display.printf("USB: ON (%.2fV %.0fmA)\n", vbus, ibus);
  } else {
    M5.Display.print("USB: OFF\n");
  }

  if (!calibratedOnce) {
    // 未キャリブ時：バッテリー＆USBだけ表示し、案内を追加
    M5.Display.setCursor(0, 36);
    M5.Display.println("Press A and Calibrate");
    return;  // 以降の詳細表示はスキップ
  }

  if (showDebug) {
    M5.Display.setCursor(0, 36);
    M5.Display.printf("aX:%6.2f aY:%6.2f aZ:%6.2f\n", ax, ay, az);
    M5.Display.setCursor(0, 66);
    M5.Display.printf("Roll :%7.2f\n", roll_d);
    M5.Display.printf("Pitch:%7.2f\n", pitch_d);
    M5.Display.printf("Yaw  :%7.2f\n", yaw_d);
    M5.Display.setCursor(0, 128);
    M5.Display.print("BtnA: Calibrate");
  } else {
    M5.Display.setCursor(0, 36);
    M5.Display.print("BtnA: Calibrate");
  }
}

// ---- BT出力（ay, pitch, roll ラベル付き）----
// ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
// ★ 変更点：クライアント接続中のみデータを送信するよう修正 ★
// ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
void outputBT(float ax, float ay, float az, float roll_d, float pitch_d, float yaw_d) {
  if (SerialBT.hasClient()) {
    SerialBT.printf("ay:%.3f, pitch:%.3f, roll:%.3f\n", ay, pitch_d, roll_d);
  }
}

// ---- シリアルプロッタ（ay, pitch, roll のみ + 固定線）----
void outputSerialPlotter(float ax, float ay, float az, float roll_d, float pitch_d, float yaw_d) {
  // ラベル行はsetupで送信済み
  Serial.print(ay, 3);       Serial.print(',');
  Serial.print(pitch_d, 3);  Serial.print(',');
  Serial.print(roll_d, 3);   Serial.print(',');
  Serial.print(PLOT_YMIN, 3);Serial.print(',');
  Serial.println(PLOT_YMAX, 3);
}

// ---- 区間5の移動平均 ----
void maPush(float r, float p, float y, float &r_avg, float &p_avg, float &y_avg) {
  int idx = maIdx % MA_N;

  sumRoll  -= maRoll[idx];
  sumPitch -= maPitch[idx];
  sumYaw   -= maYaw[idx];

  maRoll[idx]  = r;
  maPitch[idx] = p;
  maYaw[idx]   = y;

  sumRoll  += maRoll[idx];
  sumPitch += maPitch[idx];
  sumYaw   += maYaw[idx];

  maIdx++;
  if (maCount < MA_N) maCount++;

  float denom = (float)maCount;
  r_avg = sumRoll  / denom;
  p_avg = sumPitch / denom;
  y_avg = sumYaw   / denom;
}