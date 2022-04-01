#include "MeMegaPi.h"  
#include "Adafruit_TCS34725.h"
#include <Wire.h>
#include <VL53L0X.h>  


VL53L0X sensor;
int tofports[] = { 0 , 1, 2 };
void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();
}



void setup()
{
  attachInterrupt(digitalPinToInterrupt(2), Encoder3, CHANGE);
  attachInterrupt (digitalPinToInterrupt(18), Encoder1, CHANGE);
  attachInterrupt (digitalPinToInterrupt(3), Encoder2, CHANGE);
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


void Evac()  {

int case = 0, tri;

  tri = triangleDETECT;
  // if the triangle is directly in front
  if (tri == 1) {
    TurnL(90);
    Forward(100 cm ? );
    if (CheckWall() == 2) {

    case =  2 ;
      TurnR(90);
      VertCase();
    }
    else {

      TurnL(90); // not sure if there will be enough room...

      if (CheckWall() == 2)   {

      case = 1;
        TurnL(90);
        HorzCase();

      }
      else {

        TurnR(90);
        Forward(75 cm ? );
        if (CheckWall() == 2) {
        case = 3;
          Backwards(75 cm ? );

          HorzCase();


        }
        else {
        case = 4;

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

      case =  2 ;

        HorzCase();
      }
      else {

        TurnR(90); // not sure if there will be enough room...

        if (CheckWall() == 2)   {

        case = 1;
          TurnL(180);
          VertCase();

        }
        else {

          TurnR(180);
          Forward(75 cm ? );
          if (CheckWall() == 2) {
          case = 3;
            Backwards(75 cm ? );

            VertCase();


          }
          else {
          case = 4;

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
      case = 6;  // nah but it could also be 8, so maybe I might need to add like 4 more cases to reflec that fact ...
        TurnL(90);
        VertCase();
      }
      else {
        TurnL(90);
        if (CheckWall() == 2) {
        case = 5;
          TurnL(90);
          HorzCase();
        }
        else {
          TurnR(90);
          Backwards(however many cm);
          TurnR(90);
          Forwards( however many cm); // the thing is I dont know. Could need to radically alter code
          if (CheckWall() == 2) {
          case = 1 ;
            TurnL(90);
            HorzCase();

          }
          else {
          case = 2;
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

  keydifference = tofback - toffront;   
 
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
  tofright = sensor.readContinousMillimeters(); 

  if (tofright <= 300)
    return 1;

  else
    return 2;


}


void HorzCase() { 
  int tofscoop;  
  tcaselect(0); 
  tofscoop = sensor.readContinousMillimters(); 
  Lower(3100); 
  if(tofscoop < 130 ?){ 
    Forward(250); //to ram the ball in 
    Backwards(100);  
    Turn(100, 40, 180); // some type of a drag turn 
  } 
  else 
    Forward(100); 
  
}

void VertCase()  {
  int tofscoop; 
  
}
