#include <Arduino.h>
// #include "ROBOT.h"
#include "FloodFill.h"

// ROBOT robot = ROBOT(9.7, 6.5, 1030);

void setup()
{
  Serial.begin(9600);

  initialize();

  // robot.init();

  // robot.move_distance(19.2);

  // for(int i = 0; i < 2; i++){
  //   robot.move_distance(25);
  //   Serial.println("---------------------------------------------------------------------");
  // }

  // robot.Rotation_move_imu(90);
  // robot.Rotation_move_imu(90);
  // robot.rotate_angle(90);
  // robot.rotate_angle(90);
}

void loop()
{
  solve();

  // for(int i = 0; i < 3; i++){
  //   robot.move_distance(21.5);
  //   Serial.println("---------------------------------------------------------------------");
  // }

  // robot.Rotation_move_imu(-90);
  // robot.Rotation_move_imu(-90);
  // robot.rotate_angle(-90);
  // robot.rotate_angle(-90);

  // for(int i = 0; i < 3; i++){
  //   robot.move_distance(21.5);
  //   Serial.println("---------------------------------------------------------------------");
  // }

  // robot.Rotation_move_imu(90);
  // robot.Rotation_move_imu(90);
  // robot.rotate_angle(90);
  // robot.rotate_angle(90);
}