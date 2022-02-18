

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
int tofports[] = { 0 , 1 };
void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();
}


void setup()
{

  int i; 
  Serial.begin(9600);
  Wire.begin();  
  
  for(i = 0; i <= 1; i++)  {
  tcaselect(i); 
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous(); 
  }
}

void loop()
{ 
  
  triangleDETECT(); 
  /*int i; 
  for(i = 0; i <= 1; i++){ 
  tcaselect(i);
  Serial.print(sensor.readRangeContinuousMillimeters());
 

  Serial.println(); 
  } */ 
} 
int triangleDETECT() {
  //x = the differnce betweeen the location of the top and the bottom
  int tof1, tof2, keydifference;

  tcaselect(0);
  tof1 = sensor.readRangeContinuousMillimeters();
  tcaselect(1);
  tof2 = sensor.readRangeContinuousMillimeters();


  keydifference = tof1 - (tof2 - 150) - ((tof2 - 250)/10)    ;


  

  Serial.println(keydifference); 
  if(keydifference >= 40){ 

    Serial.println("triangle1"); 
    
  } 
  else if(keydifference <= -10){ 

    Serial.println("triangle2"); 
    
  } 
  else 
    Serial.println("flat wall"); 
  delay(100); 


/*
  if (keydifference > 50) {

    return 1; 

  }
  else if (keydifference < -50) {

    return 3;

  }
  else  {
   return 2;  
   */
   // TurnR(90);
   // if (tof1 <= 20)
    //  TurnL(180);
    // thjis is so that if it faces a wall its not a problem
  
}