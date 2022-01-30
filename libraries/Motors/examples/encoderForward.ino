#include "Motors.h"

Motor motor(0);
Motor motor2(1);

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  motor.run(-80);
  motor2.run(80);
  Serial.println(motor.getTicks());
  Serial.println(motor2.getTicks());
}
