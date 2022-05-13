#include "qtrSensors.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <LiquidCrystal_I2C.h>
#include <VL53L0X.h>
#include "Adafruit_TCS34725.h"
//#define MotorsOff
#include "Motors.h"
#include "utils.h"
#define log utils::logger
//#define LINEOFF
const int white_val = 150;

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
const float kp_orig = 0.07f;
float kp = kp_orig;
const float kd = 0.0f;
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

void check_lr_intersection(bool *left, bool *right)
{

  int sums = 0;
  const int tresh = 2100;
  for (int i = 0; i < SensorCount / 2; i++)
  {
    sums += qtr[i] - white_val;

    // Serial.print(i);
    //  Serial.print("\t");
    // Serial.println(qtr[i]);
  }
  //  true_count = 0;
  for (int i = SensorCount / 2; i < SensorCount; i++)
  {
    sums -= qtr[i] - white_val;
    // Serial.print(i);
    // Serial.print("\t");
    // Serial.println(qtr[i]);
  }
  // bool right = false;
  if (sums > tresh)
  {
    *left = true;
    lcd.setCursor(0, 3);
    lcd.print("int 1");
    // motor1.run(0);
    // motor2.run(0);
    // delay(3000);
  }
  if (sums < -tresh)
  {
    *right = true;
    lcd.setCursor(0, 3);
    lcd.print("int 2");
    //  motor1.run(0);
    // motor2.run(0);
    // delay(3000);
  }
}

bool linedetect()
{
  const float thresh = 380;
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

void print_raw_color(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
  log::println("raw color:");
  log::print(r);
  log::print('\t');
  log::print(g);
  log::print('\t');
  log::print(b);
  log::print('\t');
  log::println(c);
}

bool color_detect_black()
{
  tcaselect(4);
  uint16_t r, g, b, c;
  tcs.setInterrupt(false);
  tcs.getRawData(&r, &g, &b, &c);
  tcs.setInterrupt(true);
  print_raw_color(r, g, b, c);
  // random value right now
  if (c < 160)
  {
    return true;
  }
  return false;
}

void turn_left_to_black()
{
  // motor2.resetTicks();
  while (!color_detect_black())
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
  }
  utils::stopMotors();
}

void turn_right_to_black()
{
  while (!color_detect_black())
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
  }
  utils::stopMotors();
}

void right90(bool, int additional);

void left90(bool skip = false, int additional = 0)
{
  log::println("left90");
  motor2.resetTicks();
  while (motor2.getTicks() <= 150 && linedetect() && !skip)
  {
    utils::forward(100);
    qtr.Update();
    // Serial.println(linedetect());
    bool left = false, right = false;
    check_lr_intersection(&left, &right);
    if (right)
    {
      right90(true, 150 - motor2.getTicks());
      return;
    }
    // recheck intersection while moving forward what you see could change
  }
  while (motor2.getTicks() <= additional)
  {
    utils::forward(100);
  }
  qtr.Update();
  log::print("Linedetect: ");
  log::println(linedetect());
  if (linedetect())
    return;
  utils::stopMotors();
  delay(100);
  utils::resetTicks();
  while (motor1.getTicks() < 40)
  {
    utils::forward(-100);
  }
  turn_left_to_black();
  //utils::forwardTicks(-100,30);
}

void right90(bool skip = false, int additional = 0)
{
  log::println("right90");
  motor2.resetTicks();
  while (motor2.getTicks() <= 150 && linedetect() && !skip)
  {
    motor1.run(-100);
    motor2.run(100);
    qtr.Update();
    bool left = false, right = false;
    check_lr_intersection(&left, &right);
    if (left == true)
    {
      left90(true, 150 - motor2.getTicks());
      return;
    }
  }
  while (motor2.getTicks() <= additional)
  {
    utils::forward(100);
  }
  qtr.Update();
  log::print("Linedetect: ");
  Serial.println(linedetect());
  if (linedetect())
    return;
  utils::stopMotors();
  delay(100);
  utils::resetTicks();
  while (motor1.getTicks() < 40)
  {
    utils::forward(-100);
  }
  turn_right_to_black();
 // utils::forwardTicks(-100, 30);
}

int line_trace()
{
  qtr.Update();
  int32_t line = qtr.get_line();
  line -= 3500;
  // hack to improve line tracing
  const float boost = 0.08f;
  if (abs(line) > 2500)
  {
    kp = boost;
  }
  else
  {
    kp = kp_orig;
  }
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
  if (right == true)
  {
    // check green square
    // do the right turn
    //    return;
    //  motor1.run(0);
    //  motor2.run(0);
    motor1.stop();
    motor2.stop();
    // lcd.clear();
    // lcd.setCursor(0,0);
    // lcd.print("please! I just wanna go home");
    right90();

    // motor1.stop();
    // motor2.stop();
    //  delay(3000);
    // delay(3000);
  }
  if (left == true)
  {
    // check green square
    // do the left turn
    //  return;
    //  motor1.run(0);
    // motor2.run(0);
    motor1.stop();
    motor2.stop();
    // lcd.clear();
    // lcd.setCursor(0,0);
    //  lcd.print("please! I just wanna go home");
    left90();
    // delay(3000);
    //  motor1.stop();
    //  motor2.stop();
    //  delay(3000);
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

void stopMotor()
{
  motor1.stop();
  motor2.stop();
}

void right(int angle, int speed)
{
  float orient = 0;

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  int goal = (int)(orientationData.orientation.x + angle);
  orient = orientationData.orientation.x > angle + (goal - 360) ? orientationData.orientation.x - 360 : orientationData.orientation.x;

  if (goal > 360)
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
  stopMotor();
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
  stopMotor();
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
    log::println(majority_linedetect());
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
  while(motor1.getTicks() <= 40){
      utils::forward(-100);
  }
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
    log::println(majority_linedetect());
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
  while(motor1.getTicks() <= 40){
    utils::forward(-100);
  }
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
  log::begin();
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
  tcaselect(4);
  if (!tcs.begin())
  {
    Serial.println("error!");
  }
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
    log::print(qtr[i]);
    log::print('\t');
  }
  log::println();
  //tcaselect(1);
  distance = getDistCm() + 14;
  float org_dist = distance;
  log::println(distance);
  if(distance < 25){
    left(90,100);
    delay(500);
    log::print("dist after turn: ");
    distance = getDistCm() + 14;
    log::println(distance);
    if(distance < 30){
      left(180,100);
      delay(500);
      utils::forward(70);

      motor2.run(100 - org_dist * Kp_obs);
      motor1.run(-100 - org_dist * Kp_obs);
      delay(2000);
      
      qtr.Update();
      while(majority_linedetect() < 3){
        motor2.run(100 - org_dist * Kp_obs);
        motor1.run(-100 - org_dist * Kp_obs);
        qtr.Update();
      }
      utils::stopMotors();
      delay(100);
      utils::forward(100);
      delay(300);
      right(140,60);
      turn_right_to_black();
    }
    else{
      utils::forward(70);

      motor2.run(100 + org_dist * Kp_obs);
      motor1.run(-100 + org_dist * Kp_obs);
      delay(2000);

      qtr.Update();
      while(majority_linedetect() < 3){
        motor2.run(100 + org_dist * Kp_obs);
        motor1.run(-100 + org_dist * Kp_obs);
        qtr.Update();
      }
      utils::stopMotors();
      delay(100);
      utils::forward(100);
      delay(300);
      left(140,60);
      turn_left_to_black();
    }
  }

  tcaselect(2);
  float r, g, b;
  tcs.setInterrupt(false);
  tcs.getRGB(&r, &g, &b);
  tcs.setInterrupt(true);
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
