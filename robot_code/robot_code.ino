#include "qtrSensors.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <LiquidCrystal_I2C.h>
#include <VL53L0X.h>
#include "Adafruit_TCS34725.h"
//#define MOTORSOFF
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

#define LCD_ADDR 0x27
const int8_t SensorCount = 8;
QTRSensors qtr((const uint8_t[]){
                   A7, A8, A9, A10, A11, A12, A13, A14}, 
               SensorCount, A6);
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
VL53L0X tof;
sensors_event_t orientationData;
Motor motor1(MPORT2);
Motor motor2(MPORT1);
const float kp_orig = 0.09f;
float kp = kp_orig;
const float kd = 0.01f;
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
  const float thresh = 600;
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
  const float thresh = 420;
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
  const float tresh = 600;
  for(int i = 3; i <= 4; i++){
    if(qtr[i] > tresh){
      return true;
    }
  }
}

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
  log_println("raw color:");
  log_print(r);
  log_print('\t');
  log_print(g);
  log_print('\t');
  log_print(b);
  log_print('\t');
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
  log_println("left90");
  
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
  //utils::forwardTicks(-100,30);
  
}

void right90(bool skip = false, int additional = 0)
{
  log_println("right90");
  motor2.resetTicks();
  const int ticks_forward = 200;
  while (motor2.getTicks() <= ticks_forward && linedetect() && !skip)
  {
    motor1.run(-100);
    motor2.run(100);
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
 // utils::forwardTicks(-100, 30);
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

void right(int angle, int speed)
{
  float orient = 0;

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

void left(int angle, int speed)
{
  float orientation = 0;

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

uint8_t green_detect()
{
  tcaselect(2);
  float r, g, b;
  tcs.setInterrupt(!true);
  tcs.getRGB(&r, &g, &b);
  tcs.setInterrupt(!false);
  bool gleft = false, gright = false;
  if (g >= 100 && r < 100 && b < 100)
  {
    gleft = true;
  }
  print_color(r, g, b);
  tcaselect(3);
  tcs.setInterrupt(false);
  tcs.getRGB(&r, &g, &b);
  tcs.setInterrupt(true);
  print_color(r, g, b);

  if (g >= 100 && r < 100 && b < 100)
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
    log_println(majority_linedetect());
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
  utils::forward(90);
  delay(60);
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
    log_println(majority_linedetect());
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
  utils::forward(90);
  delay(60);
}

void green180()
{

  left(150, 100);
  turn_left_to_black();
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
  Serial.print(r);
  Serial.print('\t');
  Serial.print(g);
  Serial.print('\t');
  Serial.println(b);
}

void loop()
{

  float distance;
  const float Kp_obs = 3.5; // nice naming
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  qtr.Update(); 
  trace_line();

  for (int i = 0; i < SensorCount; i++)
  {
    log_print(qtr[i]);
    log_print('\t');
  }
  log_println();
  //tcaselect(1);
  distance = getDistCm() + 14;
  float org_dist = distance;
  log_println(distance);
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
      utils::forwardTicks(100,115);
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
      utils::forwardTicks(100,115);
      left(50,100);
      turn_left_to_black();
    }
  }

  tcaselect(2);
  float r, g, b;
  //tcs.setInterrupt(false);
  tcs.getRGB(&r, &g, &b);
  //tcs.setInterrupt(true);
  bool gleft = false, gright = false;
  if (g >= 100 && r < 100 && b < 100)
  {
    gleft = true;
  }
  print_color(r, g, b);
  tcaselect(3);
  //tcs.setInterrupt(false);
  tcs.getRGB(&r, &g, &b);
  //tcs.setInterrupt(true);
  print_color(r, g, b);

  if (g >= 100 && r < 100 && b < 100)
  {
    gright = true;
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
 // color_detect_black();
}
