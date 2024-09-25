#include <Arduino.h>
// #include "ROBOT.h"
#include "FloodFill.h"

// ROBOT robot = ROBOT(9.7, 6.5, 1060);

void setup()
{
  Serial.begin(9600);

   initialize();

  // robot.init();

  // robot.move_distance(19.2);

  // for(int i = 0; i < 2; i++){
  //   robot.move_distance(19.2);
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

  // for(int i = 0; i < 7; i++){
  //   robot.move_distance(19.2);
  //   Serial.println("---------------------------------------------------------------------");
  // }

  // robot.Rotation_move_imu(-90);
  // robot.Rotation_move_imu(-90);
  // robot.rotate_angle(-90);
  // robot.rotate_angle(-90);

  // for(int i = 0; i < 7; i++){
  //   robot.move_distance(19.2);
  //   Serial.println("---------------------------------------------------------------------");
  // }

  // robot.Rotation_move_imu(90);
  // robot.Rotation_move_imu(90);
  // robot.rotate_angle(90);
  // robot.rotate_angle(90);
  // Serial.print("front: ");
  // Serial.print(robot.isFrontWall());
  // Serial.print("right: ");
  // Serial.print(robot.isRightWall());
  // Serial.print("left: ");
  // Serial.println(robot.isLeftWall());
  // delay(100);
}