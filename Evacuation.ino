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
  int toffront, tofback, keydifference;

  tcaselect(1);
  toffront = sensor.readRangeContinuousMillimeters();
  tcaselect(2);
  tofback = sensor.readRangeContinuousMillimeters();

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

  int tof1, tof2, keydifference;

  tcaselect(0);
  tof1 = sensor.readRangeContinuousMillimeters();
  tcaselect(1);
  tof2 = sensor.readRangeContinuousMillimeters();

  if (tof1 <= 300)
    return 1;

  else
    return 2;


}


void HorzCase() { 
  //stub 
}

void VertCase()  {
  //stub 
  
}
