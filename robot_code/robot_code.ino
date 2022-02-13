#include "qtrSensors.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <LiquidCrystal_I2C.h>
#include <VL53L0X.h>
//#define MotorsOff
#include "Motors.h"

//#define LINEOFF
const int white_val = 150;

#define LCD_ADDR 0x27
const int8_t SensorCount = 8;
QTRSensors qtr((const uint8_t[]){A7, A8, A9, A10, A11, A12, A13, A14}, SensorCount, A6);
LiquidCrystal_I2C lcd(LCD_ADDR,20,4);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
VL53L0X tof;
Motor motor1(MPORT2);
Motor motor2(MPORT1);
float kp = 0.07f; //some random number for now
const float kd = 0.07f;
const int base_speed = 80;

#define SerialOBJ Serial
#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__})/sizeof(int))
#define SerialPrintf(fmt, ...) \
{\
char* str = new char[strlen(fmt) + NUMARGS(__VA_ARGS__)*40 + 5]; \
sprintf(str,fmt, __VA_ARGS__); \
Serial.print(str); \
delete str; \
}
void avoid_obs(){

  
}

int prev_error = 0;

void check_lr_intersection(bool* left, bool* right){

      int sums = 0;
    const int tresh = 2000;
    for(int i = 0; i < SensorCount/2; i++){
      sums += qtr[i] - white_val;
        
      //Serial.print(i);
     // Serial.print("\t");
      //Serial.println(qtr[i]);
  }
//  true_count = 0;
  for(int i = SensorCount/2; i < SensorCount; i++){
    sums -= qtr[i] - white_val;
    //Serial.print(i);
    //Serial.print("\t");
    //Serial.println(qtr[i]);
  }
 // bool right = false;
  if(sums > tresh){
    *left  = true;
    lcd.setCursor(0,3);
    lcd.print("int 1");
    //motor1.run(0);
    //motor2.run(0);
    //delay(3000);
  }
  if(sums < -tresh){
    *right = true;
    lcd.setCursor(0,3);
    lcd.print("int 2");
  //  motor1.run(0);
   // motor2.run(0);
    //delay(3000);
  }
  
}

bool linedetect(){
  const float thresh = 500;
  bool detect = false;
  for(int i = 0; i < SensorCount; i++){
    if(qtr[i] > thresh){
      detect = true;
    }
    //Serial.println(qtr[i]);
  }
  return detect;
}

void right90(bool);

void left90(bool skip = false){
  Serial.println("left90");
  motor2.resetTicks();
  while(motor2.getTicks() <= 400 && linedetect() && !skip){
    motor1.run(-100);
    motor2.run(100);
    qtr.Update();
   //Serial.println(linedetect());
   bool left = false, right = false;
   check_lr_intersection(&left,&right);
   if(right == true){
    right90(true);
    return;
   }
   //recheck intersection while moving forward what you see could change
  }
  qtr.Update();
  Serial.print("Linedetect: ");
  Serial.println(linedetect());
  if(linedetect())
  return;
  while(!linedetect()){
    motor1.run(-100);
    motor2.run(-100); //turns lol
    qtr.Update();
  }
      Serial.println((int)qtr.get_line() - 3500);
  while(abs((int)qtr.get_line() - 3500) > 500){
    Serial.println((int)qtr.get_line() - 3500);
    motor1.run(-100);
    motor2.run(-100);
    qtr.Update();
  }
  
}

void right90(bool skip = false){
  Serial.println("right90");
  motor2.resetTicks();
  while(motor2.getTicks() <= 400 && linedetect() && !skip){
    motor1.run(-100);
    motor2.run(100);
    qtr.Update();
    bool left = false, right = false;
    check_lr_intersection(&left,&right);
    if(left == true){
      left90(true);
      return;
    }
  }
   qtr.Update();
  Serial.print("Linedetect: ");
  Serial.println(linedetect());
  if(linedetect())
    return;
  while(!linedetect()){
    motor1.run(100);
    motor2.run(100);
    qtr.Update();
  }
      Serial.println((int)qtr.get_line() - 3500);
  while(abs((int)qtr.get_line() - 3500) > 500){
    Serial.println((int)qtr.get_line() - 3500);
    motor1.run(100);
    motor2.run(100);
    qtr.Update();
  }
}

void trace_line(){
  qtr.Update();
  bool right = false, left = false;
  check_lr_intersection(&left,&right);
  int32_t line = qtr.get_line();
  line -= 3500;
  //hack to improve line tracing
  const float boost = 0.1f;
  if(abs(line) > 2500){
    kp = boost;
  }
  int error = (kp * line);
  #ifndef LINEOFF
  motor1.run(-base_speed + error + ((error - prev_error) * kd));
  motor2.run(base_speed + error + ((error - prev_error) * kd));
  #endif
  lcd.setCursor(0,0);
  lcd.print(left);
  lcd.print(',');
  lcd.print(right);
  //Serial.println(error-prev_error);
    prev_error = error;
  if(right == true){
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
  if(left == true){
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
  lcd.clear();
}

void lcd_display_qtr(){
  lcd.setCursor(2,0);
  for(int i = 0; i < SensorCount; i++){
  lcd.print(qtr[i]);
  lcd.print(',');
  }
  //delay(150);
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3,0);
  //lcd.print("Hello World!");
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  tof.setTimeout(500);
  tof.init();
  tof.startContinuous();
}

void loop() {
  // put your main code here, to run repeatedly:
//  if(tof.readRangeContinuousMillimeters() < 200){
//     avoid_obs();
//  }

//SerialPrintf("dist %d\n",tof.readRangeContinuousMillimeters());
  trace_line();
  //Serial.println(motor2.getTicks());
 // qtr.Update();
  //Serial.println(linedetect());
 // lcd_display_qtr();
  
}
