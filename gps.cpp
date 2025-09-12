pros::Gps gps1(0);
pros::Gps gps2(0);
pros::Gps gps3(0);
pros::Gps gps4(0);

vector<double> calcavgPos(int numberOfSensors) {
  vector<double> currentPOS;
  double currentX;
  double currentY;
  double gps1Valx = 0;
  double gps1Valy = 0;
  double gps2Valx = 0;
  double gps2Valy = 0;
  double gps3Valx = 0;
  double gps3Valy = 0;
  double gps4Valx = 0;
  double gps4Valy = 0;

  while(true){
    if(numberOfSensors > 0){
      gps1Valx = gps1.get_position_x();
      gps1Valy = gps1.get_position_y();
      if(numberOfSensors > 1){
        gps2Valx = gps2.get_position_x();
        gps2Valy = gps2.get_position_y();
        if(numberOfSensors > 2){
          gps3Valx = gps3.get_position_x();
          gps3Valy = gps3.get_position_y();
          if(numberOfSensors > 3){
            gps4Valx = gps4.get_position_x();
            gps4Valy = gps4.get_position_y();
          }
        }
      }
    }
  }

  currentX = (gps1Valx + gps2Valx + gps3Valx + gps4Valx) / numberOfSensors;

  currentPOS.assign({currentX, currentY});

  return currentPOS;
}