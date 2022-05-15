#include <HID.h>

#include "qtrSensors.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#include <LiquidCrystal_I2C.h>
#include <VL53L0X.h>
#include "Adafruit_TCS34725.h"
//#define MotorsOff
#include "Motors.h"
#include "utils.h"

//#define LINEOFF
const int white_val = 150;
const int pingPin = A15;

#define LCD_ADDR 0x27
const int8_t SensorCount = 8;
QTRSensors qtr((const uint8_t[]) {
  A7, A8, A9, A10, A11, A12, A13, A14
}, SensorCount, A6);
//LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
VL53L0X tof;
sensors_event_t orientationData;

Motor motor1(MPORT2);
Motor motor2(MPORT1);
//TURN STRING CLOCKWISE
Motor motor3(MPORT3);

float kp = 0.07f; //some random number for now
const float kd = 0.07f;
const int base_speed = 80;

#define SerialOBJ Serial

#define MUXADDR 0x70

void tcaselect(uint8_t i) {

  if (i > 7) return;

  Wire.beginTransmission(MUXADDR);

  Wire.write(1 << i);

  Wire.endTransmission();

}

int prev_error = 0;

float getDistCm() {
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  return pulseIn(pingPin, HIGH) / 26 / 2;

}


void check_lr_intersection(bool* left, bool* right) {

  int sums = 0;
  const int tresh = 2000;
  for (int i = 0; i < SensorCount / 2; i++) {
    sums += qtr[i] - white_val;

    //Serial.print(i);
    // Serial.print("\t");
    //Serial.println(qtr[i]);
  }
  //  true_count = 0;
  for (int i = SensorCount / 2; i < SensorCount; i++) {
    sums -= qtr[i] - white_val;
    //Serial.print(i);
    //Serial.print("\t");
    //Serial.println(qtr[i]);
  }
  // bool right = false;
  if (sums > tresh) {
    *left  = true;
    //lcd.setCursor(0, 3);
    //lcd.print("int 1");
    //motor1.run(0);
    //motor2.run(0);
    //delay(3000);
  }
  if (sums < -tresh) {
    *right = true;
    //lcd.setCursor(0, 3);
    //lcd.print("int 2");
    //  motor1.run(0);
    // motor2.run(0);
    //delay(3000);
  }

}

bool linedetect() {
  const float thresh = 380;
  bool detect = false;
  for (int i = 0; i < SensorCount; i++) {
    if (qtr[i] > thresh) {
      detect = true;
    }
    //Serial.println(qtr[i]);
  }
  return detect;
}

int majority_linedetect() {
  const float thresh = 380;
  int line = 0;
  for (int i = 0; i < SensorCount; i++) {
    Serial.print(qtr[i]);
    Serial.print("     ");
    if (qtr[i] > thresh) {
      line++;
    }
    Serial.println();
  }
  return line;
}

int silver_linedetect() {
  const float thresh = 130;
  int line = 0;
  for (int i = 0; i < SensorCount; i++) {
    Serial.print(qtr[i]);
    Serial.print("     ");
    if (qtr[i] < thresh) {
      line++;
    }
  }
  Serial.println();
  return line;
}



void right90(bool, int additional);

void left90(bool skip = false, int additional = 0) {
  Serial.println("left90");
  motor2.resetTicks();
  while (motor2.getTicks() <= 150 && linedetect() && !skip) {
    utils::forward(100);
    qtr.Update();
    //Serial.println(linedetect());
    bool left = false, right = false;
    check_lr_intersection(&left, &right);
    if (right == true) {
      right90(true, 150 - motor2.getTicks());
      return;
    }
    //recheck intersection while moving forward what you see could change
  }
  while (motor2.getTicks() <= additional) {
    utils::forward(100);
  }
  qtr.Update();
  Serial.print("Linedetect: ");
  Serial.println(linedetect());
  if (linedetect())
    return;
  while (!linedetect()) {
    motor1.run(-100);
    motor2.run(-100); //turns lol
    qtr.Update();
  }
  Serial.println((int)qtr.get_line() - 3500);
  while (abs((int)qtr.get_line() - 3500) > 500) {
    Serial.println((int)qtr.get_line() - 3500);
    motor1.run(-100);
    motor2.run(-100);
    qtr.Update();
  }

}

void right90(bool skip = false, int additional = 0) {
  Serial.println("right90");
  motor2.resetTicks();
  while (motor2.getTicks() <= 150 && linedetect() && !skip) {
    motor1.run(-100);
    motor2.run(100);
    qtr.Update();
    bool left = false, right = false;
    check_lr_intersection(&left, &right);
    if (left == true) {
      left90(true, 150 - motor2.getTicks());
      return;
    }
  }
  while (motor2.getTicks() <= additional) {
    utils::forward(100);
  }
  qtr.Update();
  Serial.print("Linedetect: ");
  Serial.println(linedetect());
  if (linedetect())
    return;
  while (!linedetect()) {
    motor1.run(100);
    motor2.run(100);
    qtr.Update();
  }
  Serial.println((int)qtr.get_line() - 3500);
  while (abs((int)qtr.get_line() - 3500) > 500) {
    Serial.println((int)qtr.get_line() - 3500);
    motor1.run(100);
    motor2.run(100);
    qtr.Update();
  }
}



void trace_line() {
  qtr.Update();
  bool right = false, left = false;
  check_lr_intersection(&left, &right);
  int32_t line = qtr.get_line();
  line -= 3500;
  //hack to improve line tracing
  const float boost = 0.12f;
  if (abs(line) > 2500) {
    kp = boost;
  }
  else {
    kp = 0.07f;
  }
  int error = (kp * line);
#ifndef LINEOFF
  motor1.run(-base_speed + error + ((error - prev_error) * kd));
  motor2.run(base_speed + error + ((error - prev_error) * kd));
#endif
  //lcd.setCursor(0, 0);
  //lcd.print(left);
  //lcd.print(',');
  //lcd.print(right);
  //Serial.println(error-prev_error);
  prev_error = error;
  if (right == true) {
    //check green square
    //do the right turn
    //   return;
    // motor1.run(0);
    // motor2.run(0);
    motor1.stop();
    motor2.stop();
    //lcd.clear();
    //lcd.setCursor(0,0);
    //lcd.print("please! I just wanna go home");
    right90();

    // motor1.stop();
    // motor2.stop();
    //  delay(3000);
    //delay(3000);
  }
  if (left == true) {
    //check green square
    //do the left turn
    // return;
    // motor1.run(0);
    //motor2.run(0);
    motor1.stop();
    motor2.stop();
    //lcd.clear();
    //lcd.setCursor(0,0);
    // lcd.print("please! I just wanna go home");
    left90();
    //delay(3000);
    // motor1.stop();
    // motor2.stop();
    // delay(3000);
  }
  //lcd.clear();
}

void lcd_display_qtr() {
  //lcd.setCursor(2, 0);
  for (int i = 0; i < SensorCount; i++) {
    //lcd.print(qtr[i]);
    //lcd.print(',');
  }
  //delay(150);
}

void right(int angle, int speed) {
  tcaselect(0);

  float orient = 0;

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  int goal = (int)(orientationData.orientation.x + angle);
  orient = orientationData.orientation.x > angle + (goal - 360) ?  orientationData.orientation.x - 360 : orientationData.orientation.x ;

  Serial.print("Goal: ");
  Serial.println(goal);
  Serial.print("orient: ");
  Serial.println(orient);
  Serial.print("orientationData:  ");
  Serial.println(orientationData.orientation.x);
  Serial.println();

  if (goal >= 360) {

    goal -= 360;
    while (orient < goal) {
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      orient = orientationData.orientation.x > angle + goal ?  orientationData.orientation.x - 360 : orientationData.orientation.x ;
      //Serial.println(orient);
      motor2.run(speed);
      motor1.run(speed);
    }
  }

  else {
    while ((int)orientationData.orientation.x  < goal) {
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      motor2.run(speed);
      motor1.run(speed);
    }

  }
  utils::stopMotors();
}

void left(int angle, int speed) {
  tcaselect(0);

  float orientation = 0;


  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  int goal = (int)(orientationData.orientation.x - angle);
  orientation = orientationData.orientation.x < goal + 360 - angle ?  orientationData.orientation.x + 360 : orientationData.orientation.x;

  if (goal < 0) {

    goal += 360;

    while (orientation > goal) {
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      orientation = orientationData.orientation.x < goal - angle ?  orientationData.orientation.x + 360 : orientationData.orientation.x;
      motor2.run(-speed);
      motor1.run(-speed);

    }
  }

  else {

    while ((int)orientationData.orientation.x  > goal) {
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      motor2.run(-speed);
      motor1.run(-speed);
    }

  }
  utils::stopMotors();
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  delay(1000);

  //  qtr.calibrate(func);
  //for(int i = 0; i < SensorCount; i++)
  //Serial.println(qtr.getOffValues()[i]);
  //qtr.addOffValues((const int[]) {
  //-37, 47, 47, 47, 47, 7, -37, -121
  //});
  //lcd.init();
  //lcd.backlight();
  //lcd.setCursor(3, 0);
  //lcd.print("Hello World!");
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS); 
  
  tcaselect(0);
  tof.setTimeout(500);
  tof.init();
  tof.startContinuous();
  tcaselect(5);
  tof.setTimeout(500);
  tof.init();
  tof.startContinuous();
  tcaselect(6);
  tof.setTimeout(500);
  tof.init();
  tof.startContinuous();

  
  tcaselect(1);
  if (!tcs.begin())
  {
    Serial.println("error!");
  }
  tcaselect(2);
  if (!tcs.begin())
  {
    Serial.println("error!");
  }
  
  utils::setMotors(&motor1, &motor2);
  motor1.addBoost(20);
  motor2.addBoost(20);
}

void print_color(float r, float g, float b) {
  Serial.print(r);
  Serial.print('\t');
  Serial.print(g);
  Serial.print('\t');
  Serial.println(b);
}

void driveDist(int encoders, int speed)
{
  utils::resetTicks();

  while (abs(motor1.getTicks()) < abs(encoders) && abs(motor2.getTicks()) < abs(encoders)) {
    utils::forward(speed);
  }

  utils::stopMotors();
  return;
}

void Raise(int target)
{
 // motor3.resetTicks();
  int start = motor3.getTicks();
  while (abs(motor3.getTicks() - start) < target) {

    motor3.run(-200);

  }
  motor3.stop();
  return;
}

void Lower(int target)
{
  //motor3.resetTicks();
  int start = motor3.getTicks();
  while (abs(motor3.getTicks() - start) < target) {

    motor3.run(200);

  }
  motor3.stop();
  return;
}

int readDist() {
  tcaselect(5);
  return tof.readRangeContinuousMillimeters();
}

void loop() {

  float distance;
  float Kp_obs = 3.5;
  int pos;
  int room_orientation;
  bool leave = false;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  //Serial.println(readDist());



  qtr.Update();
  

    //triangleDETECT();
  /*
  if(silver_linedetect() >= 6){
    Serial.println("Silver detected");
    return;
    }*/

  //if (silver_linedetect() > 6) {
    Serial.println("Silver detected");
    findPosition(&pos, &room_orientation);
    Serial.println(pos);
    Serial.println(room_orientation);
    Lower(500); //scoop is going to be raised while finding the triangle so we need to put it down while we go for the ball 
    /*
    if (room_orientation == 1 && pos != 0) {
      left(90, 150);
      while (!checkWall(0, 150)) { //checkwall will probably need to be changed once we use the actual robot, because the tof sensor is located inside of the scoop  
        utils::forward(100);
      }
      right(180, 150); 
      //what is going on here? 
    }
    else if (room_orientation == 2 && pos != 0) {
      right(90, 150);
      while (!checkWall(0, 150)) {
        utils::forward(100);
      }
      left(180, 150);
    }*/
    if (room_orientation == 1) {
      int count  = 0;
      while (!leave) {
        while (!checkWall(0, 150)) {
          utils::forward(100);
        }
        if (count % 2) {
          driveDist(100, 100); 
          right(90, 150);
          driveDist(250, 100);
          Raise(600);
          leave = triangleDETECT();
          Lower(1500);
          right(90, 150);
        }
        else { 
          driveDist(100, 100); 
          left(90, 150);
          driveDist(250, 100);
          Raise(600);
          leave = triangleDETECT();
          Lower(1500);
          left(90, 150);
        }
        count++;
      }
    }
  else if (room_orientation == 2) {
      int count  = 0;
      while (!leave) {
        while (!checkWall(0, 150)) {
          utils::forward(100);
        }
        if (count % 2) { 
          driveDist(100, 100);  //needs to ram the ball in 
          left(90, 150);
          driveDist(250, 100);
          Raise(600);
          leave = triangleDETECT();
          Lower(1500);
          left(90, 150);
        }
        else {
          driveDist(100, 100); 
          right(90, 150);
          driveDist(250, 100);
          Raise(600);
          leave = triangleDETECT();
          Lower(1500);
          right(90, 150);
        }
        count++;
      }
    }
    //basically 
    Raise(0); //half way raise 
    if(triangleDETECT() != 0){ 
        left(90, 150); 
        if(checkWall(5, 30)){ 
           right(90, 150);  
           right(45, 150); 
           driveDist(250, 100);  
           right(45, 150);
           Raise(0); 
         
        } 
        else { 
           right(90, 150); 
           left(45, 150); 
           driveDist(250, 100);  
           left(45, 150);
           Raise(0); 
        
        } 
        
    }
    else {
       left(90, 150); 
       if(triangleDETECT() == 1) {
           right(180, 150); 
           while(!checkWall(5, 100))
           utils::forward(100); 
           left(180, 150); // turns left because we wouldn't want the scoop to hit the wall while half way downw. thought of this after writing it  
           Raise(0); 
          
        } 
       else {
       right(180, 150); 
        while(!checkWall(6, 100))
        utils::forward(100);  
        right(180, 150);  
        Raise(0); 
      
       }
    }
   //ryan - honestly idk if my shit makes any sense but basically it orients the robot the exact same each time it drops teh balls now, which might be useful in the situation of finding the exit 

    float future = orientationData.orientation.x + 180.0 < 360.0 ? orientationData.orientation.x : orientationData.orientation.x - 180.0;
    
    while(orientationData.orientation.x < future){
      while (!checkWall(5,1200)){
        motor1.run(150);
        motor2.run(150);
      }

      do {
        utils::resetTicks();
        utils::forward(100);

      } while(silver_linedetect() > 6 && !front_green());

      if (front_green()){
        break;
      }

      while(motor1.getTicks() > 0){
        utils::forward(-100);
      }

    
    }
  while (true);
//}

}

bool front_green(){
  tcaselect(0);

  uint16_t r, g, b, c;
  tcs.setInterrupt(false);
  tcs.getRawData(&r, &g, &b, &c);
  tcs.setInterrupt(true);

  return g >= 100 && r < 100 && b < 100;
}

void findPosition(int* triangle_pos, int* room_orient) {
  int triangle_orient;

  driveDist(400, 100);
  *room_orient = triangleDETECT();
  driveDist(400, -100);

  if (triangle_orient) {
    *triangle_pos = 1;
    return;
  }

  *room_orient = 2;
  left(90, 150);

  if (checkWall(5, 100)) {
    right(180, 150);
    *room_orient = 1;
  }

  driveDist(400, 100);
  triangle_orient = triangleDETECT();
  driveDist(400, -100);

  if (triangle_orient) {
    *triangle_pos = 2;
    *room_orient = triangle_orient; 
    return; 
  }
  else {
    *triangle_pos = 3; 
    return; 
  }


}
//numbers used for checkwall might need to be changed too because tof is inside the scoop 
bool checkWall(int sensor, int dist) {

  int tof_dist;
  // the scoop tof is located at 0  
  // the other tofs are at 5, 6 
  tcaselect(sensor);
  tof_dist = tof.readRangeContinuousMillimeters();

  return tof_dist <= dist;


}

int triangleDETECT() {

  int tofright, tofleft, keydifference;

  tcaselect(5);
  tofright = tof.readRangeContinuousMillimeters();
  tcaselect(6);
  tofleft = tof.readRangeContinuousMillimeters();

  Serial.print(tofleft);
  Serial.print("\t");
  Serial.println(tofright);
  
  keydifference = tofright - tofleft;   //WARNING: THIS IS A GUESS

  Serial.println(keydifference);
  if (keydifference >= 40) {

    Serial.println("triangle1"); 
    return (1);

  }
  else if (keydifference <= -40) {

    Serial.println("triangle2");
    return (2);

  }
  else{
    Serial.println("flat wall");
    return (0);
  }

}

// put your main code here, to run repeatedly:
//  if(tof.readRangeContinuousMillimeters() < 200){
//     avoid_obs();
//  }

/*//SerialPrintf("dist %d\n",tof.readRangeContinuousMillimeters());

  trace_line();

  /* for (int i = 0; i < SensorCount; i++) {
  Serial.print(qtr[i]);
  Serial.print('\t');
  }
  Serial.println();
  tcaselect(1);
  distance = getDistCm() + 14;
  Serial.println(distance);
  if (getDistCm() < 6) {
  left(90, 100);
  delay(500);
  Serial.print("Distance After turn: ");
  Serial.println(getDistCm());
  if (getDistCm() < 16) {
    left(180, 100);
    delay(500);
    utils::forward(70);

    motor2.run(100 - distance * Kp_obs);
    motor1.run(-100 - distance * Kp_obs);
    delay(2000);

    qtr.Update();
    while (majority_linedetect() < 3) {

      motor2.run(100 - distance * Kp_obs);
      motor1.run(-100 - distance * Kp_obs);
      qtr.Update();

    }

    utils::forward(50);
    delay(500);

    right(70, 70);
  }
  else {

    utils::forward(70);

    motor2.run(100 + distance * Kp_obs);
    motor1.run(-100 + distance * Kp_obs);
    delay(2000);



    qtr.Update();
    while (majority_linedetect() < 3) {

      motor2.run(100 + distance * Kp_obs);
      motor1.run(-100 + distance * Kp_obs);
      qtr.Update();

    }

    utils::forward(50);
    delay(500);

    left(70, 70);
  }
  }
  }
*/
/* 
void loop() {
  Raise(0); //half way raise 
    if(triangleDETECT() != 0){ 
        left(90, 150); 
        if(checkWall(5, 100)){ 
           right(90, 150);  
           right(45, 150); 
           driveDist(500, 100);  
           right(45, 150);
           Raise(600); 
         
        } 
        else { 
           right(90, 150); 
           left(45, 150); 
           driveDist(500, 100);  
           left(45, 150);
           Raise(600); 
        
        } 
        
    }
    else {
       left(90, 150); 
       if(triangleDETECT() == 1) {
           right(180, 150); 
           while(!checkWall(5, 100))
           utils::forward(100); 
           left(180, 150); // turns left because we wouldn't want the scoop to hit the wall while half way downw. thought of this after writing it  
           Raise(600); 
          
        } 
       else {
       right(180, 150); 
        while(!checkWall(6, 100))
        utils::forward(100);  
        right(180, 150);  
        Raise(600); 
      
       }
    }
   //ryan - honestly idk if my shit makes any sense but basically it orients the robot the exact same each time it drops teh balls now, which might be useful in the situation of finding the exit 

    float future = orientationData.orientation.x + 180.0 < 360.0 ? orientationData.orientation.x : orientationData.orientation.x - 180.0;
    
    while(orientationData.orientation.x < future){
      while (!checkWall(5,1200)){
        motor1.run(150);
        motor2.run(150);
      }

      do {
        utils::resetTicks();
        utils::forward(100);

      } while(silver_linedetect() > 6 && !front_green());

      if (front_green()){
        break;
      }

      while(motor1.getTicks() > 0){
        utils::forward(-100);
      }

    
    }
  while (true);


}
*/ 
// test section for exit and dropping balls. 
