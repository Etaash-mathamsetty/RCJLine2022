#include "Motors.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define LOWER 0
#define HIGHER 1

Motor motor(MPORT1);
Motor motor2(MPORT2);
VL53L0X sensor;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

const int motorSpeed = 100;

void move(int rightSpeed, int leftSpeed) {
  motor.run(-rightSpeed);
  motor2.run(leftSpeed);
}

void stopMotor() {
  motor.stop();
  motor2.stop();
}

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();
}


void setup() {

  Wire.begin();
  Serial.begin(9600);
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  sensor.startContinuous();

}
void loop() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  int orient_before;
  int distance[2];


  distance[HIGHER] = sensor.readRangeContinuousMillimeters();

  //Serial.println(orientationData.orientation.x);

  Serial.println(distance[HIGHER]);


  if (distance[HIGHER] < 50) {

    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    orient_before = orientationData.orientation.x;

    if (orient_before >= 270) {
      while (orient_before - 90 < orientationData.orientation.x) {
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        Serial.println(orientationData.orientation.x);
        move(-80, 80);
      }

    }
    else {
      while (orientationData.orientation.x < orient_before + 90) {
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        Serial.println(orientationData.orientation.x);
        move(-80, 80);
      }
    }
    stopMotor();
  }
}
