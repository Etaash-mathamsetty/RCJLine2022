#include "qtrSensors.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <LiquidCrystal_I2C.h>
#include <VL53L0X.h>
#include "Adafruit_TCS34725.h"
//#define MOTORSOFF
//#define FAKE_ROBOT
#include "Motors.h"
#include "utils.h"

#define log_begin utils::logger::begin
#if (__cplusplus >= 201703L)
#define log_println utils::logger::println
#define log_print utils::logger::print
#else
#define log_print Serial.print
#define log_println Serial.println
#endif
//#define LINEOFF
const int white_val = 227;
bool evac_zone = false;
#define LCD_ADDR 0x27
const int8_t SensorCount = 8;
QTRSensors qtr((const uint8_t[]){
                   A7, A8, A9, A10, A11, A12, A13, A14}, 
               SensorCount, A6);
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);
VL53L0X tof;
sensors_event_t orientationData;
Motor motor1(MPORT2);
Motor motor2(MPORT1);
Motor motor3(MPORT3);
const float kp_orig = 0.09f;
float kp = kp_orig;
const float kd = 0.02f;
const int base_speed = 80;
const uint8_t pingPin = A15;

#define SerialOBJ Serial

#define MUXADDR 0x70

void tcaselect(uint8_t i)
{

  if (i > 7)
    return;

  Wire.beginTransmission(MUXADDR);

  Wire.write(1 << i);

  Wire.endTransmission();
}

int prev_error = 0;

float getDistCm(){
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin, LOW);

    pinMode(pingPin, INPUT);
    return pulseIn(pingPin, HIGH)/26.0/2.0;
}


bool linedetect()
{
  const float thresh = 680;
  bool detect = false;
  for (int i = 0; i < SensorCount; i++)
  {
    if (qtr[i] > thresh)
    {
      detect = true;
    }
    // Serial.println(qtr[i]);
  }
  return detect;
}

int majority_linedetect()
{
  const float thresh = 680;
  int line = 0;
  for (int i = 0; i < SensorCount; i++)
  {
    if (qtr[i] > thresh)
    {
      line++;
    }
  }
  return line;
}

bool center_linedetect(){
  const float tresh = 680;
  for(int i = 3; i <= 4; i++){
    if(qtr[i] > tresh){
      return true;
    }
  }
}

void right(int angle, int speed, int subtract_ang = 0)
{
  float orient = 0;
  angle -= subtract_ang;

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  int goal = (int)(orientationData.orientation.x + angle);
  orient = orientationData.orientation.x > angle + (goal - 360) ? orientationData.orientation.x - 360 : orientationData.orientation.x;

  if (goal >= 360)
  {

    goal -= 360;
    while (orient < goal)
    {
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      orient = orientationData.orientation.x > angle + goal ? orientationData.orientation.x - 360 : orientationData.orientation.x;
      // Serial.println(orient);
      motor2.run(speed);
      motor1.run(speed);
    }
  }

  else
  {
    while ((int)orientationData.orientation.x < goal)
    {
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      motor2.run(speed);
      motor1.run(speed);
    }
  }
  utils::stopMotors();
}

void left(int angle, int speed, int subtract_ang = 0)
{
  float orientation = 0;
  angle -= subtract_ang;

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  int goal = (int)(orientationData.orientation.x - angle);
  orientation = orientationData.orientation.x < goal + 360 - angle ? orientationData.orientation.x + 360 : orientationData.orientation.x;

  if (goal < 0)
  {

    goal += 360;

    while (orientation > goal)
    {
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      orientation = orientationData.orientation.x < goal - angle ? orientationData.orientation.x + 360 : orientationData.orientation.x;
      motor2.run(-speed);
      motor1.run(-speed);
    }
  }

  else
  {

    while ((int)orientationData.orientation.x > goal)
    {
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      motor2.run(-speed);
      motor1.run(-speed);
    }
  }
  utils::stopMotors();
}

#ifdef MOTORSOFF
#define DISABLE_INT_CHECK
#endif
//#define DISABLE_INT_CHECK

void check_lr_intersection(bool *left, bool *right)
{
  #ifdef DISABLE_INT_CHECK
  *left = false;
  *right = false;
  return;
  #endif

  const int tresh = 600;
  int triggers = 0;
  for(int i = 0; i < SensorCount/2; i++){
    if(qtr[i] >= tresh){
      triggers++;
    }
  }
  if(triggers >= 3)
    *left = true;
  triggers = 0;
  for(int i = SensorCount/2; i < SensorCount; i++){
    if(qtr[i] >= tresh)
      triggers++;
  }
  if(triggers >= 3){
    *right = true;
  }
}

void print_raw_color(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
  //log_print("raw color:");
  log_print("R: ");
  log_print(r);
  log_print("\t G: ");
  log_print(g);
  log_print("\t B: ");
  log_print(b);
  log_print("\t C: ");
  log_println(c);
}

void turn_left_to_black()
{
  // motor2.resetTicks();
  /*while (!color_detect_black())
  {
    motor1.run(-100);
    motor2.run(-100);
  }
  while(color_detect_black()){
    motor1.run(-100);
    motor2.run(-100);
  }
  while(!color_detect_black()){
    motor1.run(90);
    motor2.run(90);
  }*/
  //int32_t fast_abs(int32_t num)
  //return (num & (INT32_MAX >> 1));
  qtr.Update();
  while(!(majority_linedetect() >= 2)){
    qtr.Update();
    motor1.run(-100);
    motor2.run(-100);
  }
  while(abs(qtr.get_center()) >= 200){
    qtr.Update();
    motor1.run(-100);
    motor2.run(-100);
  }
  //small correction
  left(7, 100);
  utils::stopMotors();
}

void turn_right_to_black()
{
  /*while (!color_detect_black())
  {
    motor1.run(100);
    motor2.run(100);
  }
  while(color_detect_black()){
    motor1.run(100);
    motor2.run(100);
  }
  while(!color_detect_black()){
    motor1.run(-90);
    motor2.run(-90);
  }*/
  qtr.Update();
  while(!(majority_linedetect() >= 2)){
    qtr.Update();
    motor1.run(100);
    motor2.run(100);
  }
  while(abs(qtr.get_center()) >= 200){
    qtr.Update();
    motor1.run(100);
    motor2.run(100);
  }
  //small correction
  right(7, 100);
  utils::stopMotors();
}

void right90(bool, int additional);

void left90(bool skip = false, int additional = 0)
{
  //log_println("left90");
  
  motor2.resetTicks();
  const int ticks_forward = 200;
  while (motor2.getTicks() <= ticks_forward && linedetect() && !skip)
  {
    utils::forward(100);
    qtr.Update();
    // Serial.println(linedetect());
    bool left = false, right = false;
    check_lr_intersection(&left, &right);
   if(left && right){
      return;
    }
    else if (right)
    {
      right90(true, ticks_forward - motor2.getTicks());
      return;
    }
    // recheck intersection while moving forward what you see could change
  }
  while (motor2.getTicks() <= additional && linedetect())
  {
    utils::forward(100);
  }
  utils::stopMotors();
  delay(100);
  qtr.Update();
  if(linedetect())
    return;
  utils::resetTicks();
  utils::forwardTicks(100, 50);
  turn_left_to_black();
  utils::forwardTicks(-100,50); 
}

void right90(bool skip = false, int additional = 0)
{
  //log_println("right90");
  motor2.resetTicks();
  const int ticks_forward = 200;
  while (motor2.getTicks() <= ticks_forward && linedetect() && !skip)
  {
    utils::forward(100);
    qtr.Update();
    bool left = false, right = false;
    check_lr_intersection(&left, &right);
    if(left && right)
      return;
    else if (left == true)
    {
      left90(true, ticks_forward - motor2.getTicks());
      return;
    }
  }
  while (motor2.getTicks() <= additional && linedetect())
  {
    utils::forward(100);
  }
  utils::stopMotors();
  delay(100);
  qtr.Update();
  if(linedetect())
    return;
  utils::resetTicks();
  utils::forwardTicks(100,50);
  turn_right_to_black();
  utils::forwardTicks(-100, 50);
}

int line_trace()
{
  qtr.Update();
  int32_t line = qtr.get_line();
  line -= 3500;
  // hack to improve line tracing (not anymore)
 /* const float boost = 0.1f;
  if (abs(line) > 2500)
  {
    kp = boost;
  }
  else
  {
    kp = kp_orig;
  }*/
  int error = (kp * line);
#ifndef LINEOFF
  motor1.run(-base_speed + error + ((error - prev_error) * kd));
  motor2.run(base_speed + error + ((error - prev_error) * kd));
#endif
  return error;
}

void trace_line()
{
  qtr.Update();
  bool right = false, left = false;
  check_lr_intersection(&left, &right);
  int error = line_trace();
  lcd.setCursor(0, 0);
  lcd.print(left);
  lcd.print(',');
  lcd.print(right);
  // Serial.println(error-prev_error);
  prev_error = error;
  if(right && left){
    utils::forwardTicks(100, 100);
  }
  else if (right == true)
  {
    utils::stopMotors();
    right90();
  }
  else if (left == true)
  {
    utils::stopMotors();    
    left90();
  }
  lcd.clear();
}

void lcd_display_qtr()
{
  lcd.setCursor(2, 0);
  for (int i = 0; i < SensorCount; i++)
  {
    lcd.print(qtr[i]);
    lcd.print(',');
  }
  // delay(150);
}

uint8_t green_detect()
{
  tcaselect(2);
  float r, g, b;
  //tcs.setInterrupt(!true);
  tcs.getRGB(&r, &g, &b);
  //tcs.setInterrupt(!false);
  bool gleft = false, gright = false;
  Serial.println(g/b * 10);
  if ((g/b) * 10 >= 11)
  {
    gleft = true;
  }
  //print_color(r, g, b);
  tcaselect(3);
  //tcs.setInterrupt(false);
  tcs.getRGB(&r, &g, &b);
  //tcs.setInterrupt(true);
  //print_color(r, g, b);
  Serial.println(g/b * 10);
  if ((g/b) * 10 >= 11)
  {
    gright = true;
  }

  return (uint8_t(gleft * UINT8_MAX) & 0x0F) | (uint8_t(gright * UINT8_MAX) & 0xF0);
}

void green180();

void green90l()
{
  motor2.resetTicks();
  // forward
  uint8_t double_green = 0;
  bool pls_return = false;
  while (motor2.getTicks() <= 5)
  {
    utils::forward(80);
    qtr.Update();
    //log_println(majority_linedetect());
    double_green = green_detect();
    if (double_green == 0xFF)
    {
      green180();
      return;
    }
    if (majority_linedetect() >= 4)
    {
      pls_return = true;
    }
  }

  while (motor2.getTicks() <= 150)
  {
    utils::forward(100);
  }
  if (pls_return)
  {
    return;
  }
  utils::stopMotors();
  delay(200);
  utils::resetTicks();
 // while(motor1.getTicks() <= 40){
  //    utils::forward(-100);
  //}
  utils::forwardTicks(100,50);
  utils::stopMotors();
  delay(60);
  left(60, 100);
  turn_left_to_black();
  utils::forwardTicks(-100,50);
}

void green90r()
{
  motor2.resetTicks();
  // forward
  uint8_t double_green = 0;
  bool pls_return = false;
  while (motor2.getTicks() <= 5)
  {
    utils::forward(80);
    qtr.Update();
    //log_println(majority_linedetect());
    double_green = green_detect();
    if (double_green == 0xFF)
    {
      green180();
      return;
    }
    if (majority_linedetect() >= 4)
    {
      pls_return = true;
    }
  }

  while (motor2.getTicks() <= 150)
  {
    utils::forward(100);
  }
  if (pls_return)
  {
    return;
  }
  utils::stopMotors();
  delay(200);
  utils::resetTicks();
  //while(motor1.getTicks() <= 40){
  //  utils::forward(-100);
  //}
  utils::forwardTicks(100,50);
  utils::stopMotors();
  delay(60);
  right(60, 100);
  turn_right_to_black();
  utils::forwardTicks(-100,50);
}

void green180()
{

  left(150, 100);
  turn_left_to_black();
}

void driveDist(int encoders, int speed, int reset = true)
{
  if (reset)
    utils::resetTicks();

  while (abs(motor1.getTicks()) < abs(encoders) && abs(motor2.getTicks()) < abs(encoders)) {
    utils::forward(speed);
  }

  utils::stopMotors();
  return;
}

void Raise(int target)
{

#ifndef FAKE_ROBOT
  // motor3.resetTicks();
  int start = motor3.getTicks();
  while (abs(motor3.getTicks() - start) < target) {

    motor3.run(-200);

  }
  motor3.stop();
  return;
#endif
}

void Lower(int target)
{
#ifndef FAKE_ROBOT
  //motor3.resetTicks();
  int start = motor3.getTicks();
  while (abs(motor3.getTicks() - start) < target) {

    motor3.run(200);

  }
  motor3.stop();
  return;
#endif
}

bool front_green() {


#ifndef FAKE_ROBOT
  return green_detect() > 0;
#endif
#ifdef FAKE_ROBOT
  return false;
#endif
}




void setup()
{
  // put your setup code here, to run once:
  Wire.begin();
  log_begin();
  delay(1000);
  const int boost = 10;
  motor1.addBoost(boost);
  motor2.addBoost(boost);
  //  qtr.calibrate(func);
  // for(int i = 0; i < SensorCount; i++)
  // Serial.println(qtr.getOffValues()[i]);
  qtr.addOffValues((const int[]){
      -37, 47, 47, 47, 47, 7, -37, -121});
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);
  // lcd.print("Hello World!");
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
  
  tcaselect(2);
  if (!tcs.begin())
  {
    Serial.println("error first!");
  }
  tcaselect(3);
  if (!tcs.begin())
  {
    Serial.println("error!");
  }
 // tcaselect(4);
 // if (!tcs.begin())
 // {
 //   Serial.println("error!");
 // }
  utils::setMotors(&motor1, &motor2);
}

void print_color(float r, float g, float b)
{
  log_print(r);
  log_print('\t');
  log_print(g);
  log_print('\t');
  log_println(b);
}

int silver_persistance = 0;

void loop()
{
if(!evac_zone){
  uint16_t r1,g1,b1,r2,g2,b2, c1 = 0, c2 = 0;
//print_raw_color(r1,g1,b1,c1);
tcaselect(2);
tcs.getRawData(&r1, &g1, &b1, &c1);
tcaselect(3);
tcs.getRawData(&r2, &g2, &b2, &c2);
//print_raw_color(r2, g2, b2, c2);
//Serial.print("qtr[4]: ");
//Serial.println(qtr[4]);
//print_raw_color(r2,g2,b2,c2);
Serial.print("r/g:");
Serial.println((r2/(float)g2) * 10);
//Serial.print(" c:");
//Serial.print(c2);
if(c2 >=  950 && (r2/(float)g2) * 10 >= 10.5){

  silver_persistance++;
  log_println("silver");
}
else{
  silver_persistance = 0;
}

if(silver_persistance >= 2){
    utils::stopMotors();
    evac_zone = true;
    return;
}
  float distance;
  const float Kp_obs = 3.5; // nice naming
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  qtr.Update(); 
  trace_line();

  //for (int i = 0; i < SensorCount; i++)
  //{
  //  log_print(qtr[i]);
  //  log_print('\t');
  //}
  log_println();
  //tcaselect(1);
  #ifndef MOTORSOFF
  distance = getDistCm() + 14;
  //log_println(distance);
  #else
  distance = 50;
  #endif
  float org_dist = distance;
  if(distance < 25){
    left(90,120);
    delay(500);
    log_print("dist after turn: ");
    distance = getDistCm() + 14;
    log_println(distance);
    if(distance < 30){
      left(180,100);
      delay(500);
      utils::forward(70);

      motor2.run(100 - org_dist * Kp_obs);
      motor1.run(-100 - org_dist * Kp_obs);
      delay(2500);
      
      qtr.Update();
      while(majority_linedetect() < 3){
        motor2.run(100 - org_dist * Kp_obs);
        motor1.run(-100 - org_dist * Kp_obs);
        qtr.Update();
      }
      utils::stopMotors();
      delay(100);
      utils::forwardTicks(100,125);
      right(50,100);
      turn_right_to_black();
    }
    else{
      utils::forward(70);

      motor2.run(100 + org_dist * Kp_obs);
      motor1.run(-100 + org_dist * Kp_obs);
      delay(2500);

      qtr.Update();
      while(majority_linedetect() < 3){
        motor2.run(100 + org_dist * Kp_obs);
        motor1.run(-100 + org_dist * Kp_obs);
        qtr.Update();
      }
      utils::stopMotors();
      delay(100);
      utils::forwardTicks(100,125);
      left(50,100);
      turn_left_to_black();
    }
  }

  tcaselect(2);
  float r, g, b;
  //tcs.setInterrupt(false);
  tcs.getRGB(&r, &g, &b);
  //Serial.print("g/b:");
  //Serial.println((g/b) * 10);
  //print_color(r, g, b);
  //tcs.setInterrupt(true);
  bool gleft = false, gright = false;
  Serial.println(green_detect(), HEX);
  if (green_detect() & 0x0F > 0)
  {
    #ifndef MOTORSOFF
    gleft = true;
    #else
    //log_println("green");
    #endif
  }
  

  tcaselect(3);
  //tcs.setInterrupt(false);
  tcs.getRGB(&r, &g, &b);
  //tcs.setInterrupt(true);
  //print_color(r, g, b);
  //Serial.print("g2/b2:");
  //Serial.println((g/b) * 10);
  if (green_detect() & 0xF0 > 0)
  {
    #ifndef MOTORSOFF
    gright = true;
    #else
    //log_println("green");
    #endif
  }

  if (gleft && gright)
  {
    // turn around
    green180();
  }
  else if (gleft)
  {
    green90l();
  }
  else if (gright)
  {
    green90r();
  }

/*
if((r2 + g2 + b2) >= 1380 && (r1 + g1 + b1) >= 1380){
  log_println("silver");
  evac_zone = true;
  utils::stopMotors();
  return;
}*/
}
else{
  
  
}
 // color_detect_black();
}
