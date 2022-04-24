//#include "MeMegaPi.h"  
#include "Adafruit_TCS34725.h"
#include <Wire.h>
#include <VL53L0X.h>  
#include <Motors.h>


VL53L0X sensor;
Motor motor1(MPORT1); 
Motor motor2(MPORT2); 
Motor motor3(MPORT3);
volatile int Enc1 = 0, Enc2 = 0;

int tofports[] = { 0 , 1, 2 };
void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();
}
void Drive(int c, int l, int r)
{ 
  while (abs(motor1.getTicks()) < c && abs(motor2.getTicks()) < c) {


    motor1.run(l);

    motor2.run(-r);

  }
  motor1.stop(); 
  motor2.stop(); 
  return;
}

int triangleDETECT();
#define TriangleDETECT() triangleDETECT()

void setup()
{
  //attachInterrupt(digitalPinToInterrupt(2), Encoder3, CHANGE);
  //attachInterrupt (digitalPinToInterrupt(18), Encoder1, CHANGE);
  //attachInterrupt (digitalPinToInterrupt(3), Encoder2, CHANGE);
  int i; 
  Serial.begin(9600);
  Wire.begin();  
  
  for(i = 0; i <= 2; i++)  {
  tcaselect(i); 
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }


  sensor.startContinuous(); 
  }
 
}

void TurnL(int deg){}
void TurnR(int deg){}
void Forward(int cm){}
void Backward(int cm){}
void VertCase(){}
void HorzCase(){}

void Evac()  {

int _case = 0, tri;

  tri = triangleDETECT();
  // if the triangle is directly in front
  if (tri == 1) {
    TurnL(90);
    Forward(100);
    if (CheckWall() == 2) {

    _case =   2 ;
      TurnR(90);
      VertCase();
    }
    else {

      TurnL(90); // not sure if there will be enough room...

      if (CheckWall() == 2)   {

      _case =  1;
        TurnL(90);
        HorzCase();

      }
      else {

        TurnR(90);
        Forward(75 cm ? );
        if (CheckWall() == 2) {
          _case =  3;
          Backwards(75 cm ? );

          HorzCase();


        }
        else {
           _case =  4;

          Backwards(75 cm ? );

          VertCase();

        }
      }
    }
  }
  else if (tri == 2) {

    TurnL(90);
    // if it stares at a wall
    if (tof1 < 300) {

      TurnR(180);

    }
    tri = TriangleDETECT();

    //triangle on the side of the robot

    if (tri == 1) {

      TurnR(90);
      Forward(100 cm ? );
      if (CheckWall() == 2) {

      _case =   2 ;

        HorzCase();
      }
      else {

        TurnR(90); // not sure if there will be enough room...

        if (CheckWall() == 2)   {

        _case =  1;
          TurnL(180);
          VertCase();

        }
        else {

          TurnR(180);
          Forward(75 cm ? );
          if (CheckWall() == 2) {
          _case =  3;
            Backwards(75 cm ? );

            VertCase();


          }
          else {
          _case =  4;

            Backwards(75 cm ? );

            HorzCase();

          }
        }
      }




    }

    // triangle in far corner case

    else {
      TurnR(90);
      if (tof1 <= 300) {
        TurnL(180);
      }
      Forward(75 ? cm);
      if (CheckWall() == 2)  {
      _case =  6;  // nah but it could also be 8, so maybe I might need to add like 4 more cases to reflec that fact ...
        TurnL(90);
        VertCase();
      }
      else {
        TurnL(90);
        if (CheckWall() == 2) {
        _case =  5;
          TurnL(90);
          HorzCase();
        }
        else {
          TurnR(90);
          Backwards(however many cm);
          TurnR(90);
          Forwards( however many cm); // the thing is I dont know. Could need to radically alter code
          if (CheckWall() == 2) {
          _case =  1 ;
            TurnL(90);
            HorzCase();

          }
          else {
          _case =  2;
            VertCase();
          }
        }
      }
    }
  }
}
int triangleDETECT() {
  //x = the differnce betweeen the location of the top and the bottom
  int tofright, tofleft, keydifference;

  tcaselect(1);
  tofright = sensor.readRangeContinuousMillimeters();
  tcaselect(2);
  tofleft = sensor.readRangeContinuousMillimeters();

  keydifference = tofright - tofleft;   //WARNING: THIS IS A GUESS
 
  Serial.println(keydifference); 
  if(keydifference >= 40){ 

    Serial.println("triangle1");  
    return(1); 
    
  } 
  else if(keydifference <= -40){ 

    Serial.println("triangle2");  
    return(2); 
    
  } 
  else 
    Serial.println("flat wall"); 
  delay(100);  
  return(3); 
 

}
int CheckWall() {

  int tofright; 

  tcaselect(2);
  tofright = sensor.readRangeContinuousMillimeters(); 

  if (tofright <= 300)
    return 1;

  else
    return 2;


}

void Navigation() { 
    // put your main code here, to run repeatedly:
  int spinnumber = 2600; 
  //will need to implement tof sensor into this, probably not that hard 
  
  Drive(2500, 100, 100); 
  delay(500); 
  Drive(500, -100, -100); 
  delay(500); 
  Drive(spinnumber, 150, 0); 
  delay(500); 
  Drive(2500, 100, 100); 
  delay(500);  
  Drive(500, -100, -100); 
  delay(500); 
  Drive(spinnumber, 0, 150); 
  delay(500);  
  
  
  //need to figure out how to detect the triangle during the evac movemnts
  //when trinagle is found, spin the robot around, back up and then dump the balls. 
  
}

