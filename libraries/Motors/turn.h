#include "Motors.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>

#define MPORT1 0
#define MPORT2 1
#define MPORT3 2
#define MPORT4 3

class turnMotor {
    public:

    Motor motor1;
    Motor motor2;
    
        turnMotor(int port1, int port2) {
            this->motor1 = Motor(port1);
            this->motor2 = Motor(port2);
        }

    void right(int angle, int speed) {
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        orient_before = orientationData.orientation.x;

        if (orient_before >= 360 - angle) {
            while (orientationData.orientation.x <= 360) {
                bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
                Serial.println(orientationData.orientation.x);
                move(-speed, speed);
            }
            while (orientationData.orientation.x < angle - (360 - orient_before) {
                bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
                Serial.println(orientationData.orientation.x);
                move(-speed, speed);
            }

        }
        else {
            while (orientationData.orientation.x < orient_before + angle) {
                bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
                Serial.println(orientationData.orientation.x);
                move(-speed, speed);
            }
        }
        stopMotor();
    }

    void left(int angle, speed) {
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        orient_before = orientationData.orientation.x;

        if (orient_before <= angle) {
            while (orientationData.orientation.x >= 0) {
                bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
                Serial.println(orientationData.orientation.x);
                move(speed, -speed);
            }
            while (orientationData.orientation.x > 360 - (angle - orient_before)){
                bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
                Serial.println(orientationData.orientation.x);
                move(speed, -speed);
            }

        }
        else {
            while (orientationData.orientation.x > orient_before - angle) {
                bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
                Serial.println(orientationData.orientation.x);
                move(speed, -speed);
            }
        }
        stopMotor();
    }


}
