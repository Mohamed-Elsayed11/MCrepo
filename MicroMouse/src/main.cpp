#include <Arduino.h>
#include "ROBOT.h"
// #include "FloodFill.h"

ROBOT robot = ROBOT(9.5, 6.5, 970);

void setup()
{
  //  while(!Serial){
  //    ;
  //  }

  Serial.begin(9600);

  // initialize();

  robot.init();
  for(int i = 0; i < 3; i++){
    robot.move_distance(19.2);
  //  Serial.println("---------------------------------------------------------------------");
  }
  delay(50);
  robot.Rotation_move_imu(90);
}

void loop()
{
  // for(int i = 0; i < 3; i++){
  //   robot.move_distance(19.2);
  // //  Serial.println("---------------------------------------------------------------------");
  // }
  //  robot.Rotation_move_imu(90);
  // for(int i = 0; i < 2; i++){
  //    robot.Rotation_move_imu(90);
  //  Serial.println("---------------------------------------------------------------------");
  // }

  // solve();
}