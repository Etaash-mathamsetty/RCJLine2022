
//untested and written in a text editor lmao
#include "Motors.h"

//#define MUXADDR 0x70

namespace utils{

Motor* motor;
Motor* motor2;

void setMotors(Motor* _motor1, Motor* _motor2){
	motor = _motor1;
	motor2 = _motor2;
}

void forward(int rightSpeed, int leftSpeed) {
  motor->run(-rightSpeed);
  motor2->run(leftSpeed);
}
	
void forward(int speed){
  forward(speed,speed);	
}

void stopMotors(){
	motor->stop();
	motor2->stop();
}

namespace logger{
	void println(){
		Serial.println();
	}
	
	template <typename T>
	void println(T thing){
		Serial.println(thing);	
	}
	
	template <typename T>
	void print(T thing){
		Serial.print(thing);	
	}
};

};
