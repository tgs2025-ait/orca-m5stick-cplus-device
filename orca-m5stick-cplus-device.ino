#include <M5StickCPlus.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
float AX, AY, AZ;

void setup() {
  M5.begin();           // デバイス初期化
  M5.IMU.Init();        // IMU初期化
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextFont(2);
  M5.Lcd.setTextSize(3);

  Serial.begin(115200); // ハードウェアシリアルを初期化
  SerialBT.begin(115200);//Bluetooth経由でのシリアル通信のボーレードを指定
  SerialBT.begin("orca-m5stick-c-plus-device");//Bluetoothデバイス名


  delay(100);           // 安定化待ち
}

void loop() {
  // 加速度データ取得
  M5.IMU.getAccelData(&AX, &AY, &AZ);

  // LCD 表示（不要ならコメントアウト）
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("X:%6.2f g\n", AX);
  M5.Lcd.printf("Y:%6.2f g\n", AY);
  M5.Lcd.printf("Z:%6.2f g\n", AZ);

  // シリアルプロッタ用出力
  // フォーマット：AX:値 AY:値 AZ:値
  Serial.print("AX:"); Serial.print(AX);
  SerialBT.print("AX:"); SerialBT.print(AX);

  Serial.print(" AY:"); Serial.print(AY);
  SerialBT.print(" AY:"); SerialBT.print(AY);

  Serial.print(" AZ:"); Serial.println(AZ);
  SerialBT.print(" AZ:"); SerialBT.println(AZ);

  delay(200);  // 出力間隔 200ms（約5Hz）
}