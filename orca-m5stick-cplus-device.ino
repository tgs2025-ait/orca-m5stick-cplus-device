#include <M5StickCPlus.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
float ax, ay, az;

void setup() {

  M5.begin();//デバイス初期化
  M5.IMU.Init();//IMU初期化

  M5.Lcd.setRotation(3);//画面回転
  M5.Lcd.fillScreen(BLACK);// 背景を黒でクリア
  M5.Lcd.setTextFont(2);   // フォントサイズ設定
  M5.Lcd.setTextSize(3);//テキストサイズ設定

  SerialBT.begin(115200);//Bluetooth経由でのシリアル通信のボーレードを指定
  SerialBT.begin("orca-m5stick-c-plus-device");//Bluetoothデバイス名

}

void loop() {

  // 加速度データ取得
  M5.IMU.getAccelData(&ax, &ay, &az);
  // 表示位置・内容
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("AX: %6.2f g\n", ax);
  M5.Lcd.printf("AY: %6.2f g\n", ay);
  M5.Lcd.printf("AZ: %6.2f g\n", az);

  SerialBT.printf("AX: %6.2f g\n", ax);
  SerialBT.printf("AY: %6.2f g\n", ay);
  SerialBT.printf("AZ: %6.2f g\n", az);

  delay(200);  // 更新間隔（200ms）
  
}