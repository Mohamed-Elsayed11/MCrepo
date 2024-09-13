#include <Arduino.h>
// #include "ROBOT.h"
// #include "IMU.h"
#include "FloodFill.h"

// IMU2040 imu = IMU2040();
// ROBOT robot = ROBOT(10.1, 6.5, 970);
// int flag = 0;
// unsigned long prev_time = 0;

void setup()
{
  //  while(!Serial){
  //    ;
  //  }

  // if (!imu.init())
  // {
  //   Serial.println("IMU initialization failed!");
  //   while (1)
  //     ;
  // }
  // else
  // {
  //   Serial.println("IMU initialized");
  // }

  Serial.begin(9600);

  initialize();

  // robot.init();

  // for(int i = 0; i < 5; i++){
  //   robot.move_distance(19.2);
  //   Serial.println("---------------------------------------------------------------------");
  // }

  // robot.rotate_angle(90);
  // robot.rotate_angle(90);
  // robot.rotate_angle(-90);

  // delay(50);
  // robot.Rotation_move_imu(90);
  // robot.Rotation_move_imu(-90);
  // prev_time = millis();
  // robot.move_distance(9);
}

void loop()
{
  // for(int i = 0; i < 5; i++){
  //   robot.move_distance(19.2);
  //   Serial.println("---------------------------------------------------------------------");
  // }

  // robot.Rotation_move_imu(90);
  // robot.Rotation_move_imu(90);

  // robot.rotate_angle(90);
  // robot.rotate_angle(90);

  solve();
}