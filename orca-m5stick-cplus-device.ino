#include <M5StickCPlus.h>

void setup() {
  M5.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.print("Hello M5!");
  Serial.begin(115200);
  Serial.println("setup end!");
}

void loop() {
  Serial.println("test");
  delay(1000);
}