#include "qtrSensors.h"

const uint8_t SensorCount = 8;
QTRSensors qtr((const uint8_t[]){3, 4, 5, 6, 7, 8, 9, 10}, SensorCount, 2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  qtr.Update();
  uint32_t value = qtr.get_line();
  Serial.println(value);
  delay(100);
}
