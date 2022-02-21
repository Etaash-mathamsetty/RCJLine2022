#include "Motors.h"
#include <VL53L0X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>

Motor motor(MPORT1);
Motor motor2(MPORT2);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
VL53L0X sensor;
sensors_event_t orientationData;

void stopMotor() {
  motor.stop();
  motor2.stop();
}

void right(int angle, int speed) {
  float orient = 0;

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  int goal = (int)(orientationData.orientation.x + angle);
  orient = orientationData.orientation.x > angle + (goal - 360) ?  orientationData.orientation.x - 360 : orientationData.orientation.x ;

  if (goal > 360) {

    goal -= 360;
    while (orient < goal) {
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      orient = orientationData.orientation.x > angle + goal ?  orientationData.orientation.x - 360 : orientationData.orientation.x ;
      //Serial.println(orient);
      motor.run(speed);
      motor2.run(speed);
    }
  }

  else {
    while ((int)orientationData.orientation.x  < goal) {
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      motor.run(speed);
      motor2.run(speed);
    }

  }
  stopMotor();
}

void left(int angle, int speed) {
  float orientation = 0;


  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  int goal = (int)(orientationData.orientation.x - angle);
  orientation = orientationData.orientation.x < goal + 360 - angle?  orientationData.orientation.x + 360: orientationData.orientation.x;

  if (goal < 0) {

    goal += 360;
    
    while (orientation > goal) {
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      orientation = orientationData.orientation.x < goal - angle?  orientationData.orientation.x + 360 : orientationData.orientation.x;
      motor.run(-speed);
      motor2.run(-speed);

    }
  }

  else {
    
    while ((int)orientationData.orientation.x  > goal) {
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      motor.run(-speed);
      motor2.run(-speed);
    }

  }
  stopMotor();
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
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
    tcaselect(1);


    sensor.setTimeout(500);
    if (!sensor.init()) {
    while (1) {}
    }
    sensor.startContinuous();
}

void loop() {

  float distance;
  float Kp = 0.28;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  while((distance = sensor.readRangeContinuousMillimeters()) > 200);
    Serial.println(sensor.readRangeContinuousMillimeters());
    left(90, 100);
    delay(500);
    Serial.println(sensor.readRangeContinuousMillimeters());
  if(sensor.readRangeContinuousMillimeters() < 200){
    left(180,100);
    delay(500);
    while(true){

      motor.run(100 - distance * Kp);
      motor2.run(-100 - distance * Kp);
    
    }
  }
  else{
     while(true){

      motor.run(100 + distance * Kp);
      motor2.run(-100 + distance * Kp);
    
    }
    
 }
    while(true);
}
