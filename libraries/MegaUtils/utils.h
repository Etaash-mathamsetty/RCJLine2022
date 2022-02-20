
//untested and written in a text editor lmao
#include "Motors.h"

#define MUXADDR 0x70

namespace utils{

Motor& motor;
Motor& motor2;

void setMotors(Motor& _motor1, Motor& _motor2){
	motor = _motor1;
	motor2 = _motor2;
}

void move(int rightSpeed, int leftSpeed) {
  motor.run(-rightSpeed);
  motor2.run(leftSpeed);
}

void stopMotors(){
	motor.stop();
	motor2.stop();
}

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(MUXADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

};