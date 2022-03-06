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
  int tof1, tof2, keydifference;

  tcaselect(0);
  tof1 = sensor.readRangeContinuousMillimeters();
  tcaselect(1);
  tof2 = sensor.readRangeContinuousMillimeters();
  keydifference = tof2 - tof1;



  if (keydifference > 50) {

    return 1; // triangle

  }
  else if (keydifference < -50) {

    return 1; // triangle

  }
  else
    return 2;  // means that there is a flat wall

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
