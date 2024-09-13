#include <Arduino.h>
#include "ROBOT.h"
// #include "IMU.h"
// #include "FloodFill.h"

// IMU2040 imu = IMU2040();
ROBOT robot = ROBOT(9.7, 6.5, 970);
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

  // initialize();

  robot.init();

  // for(int i = 0; i < 7; i++){
  //   robot.move_distance(19.2);
  //   Serial.println("---------------------------------------------------------------------");
  // }

  // robot.rotate_angle(90);
  // robot.rotate_angle(90);

  // delay(50);
  // robot.Rotation_move_imu(90);
  // prev_time = millis();
  // robot.move_distance(9);
}

void loop()
{
  // imu.calulations();
  // double yaw_angle = imu.get_Yaw_angle();
  // Serial.print("angle: ");
  // Serial.println(yaw_angle);
  // delay(100);

  // for(int i = 0; i < 3; i++){
  //   robot.move_distance(19.2);
  // //  Serial.println("---------------------------------------------------------------------");
  // }

  // for(int i = 0; i <1; i++)
  // {
  //    if(flag==0)
  //    {
  //      robot.Rotation_move_imu(90);
  //      flag=1;
  //      delay(100);
  //    }
  
  // robot.move_distance(19.2);
  // robot.move_distance(19.2);
  // robot.move_distance(19.2);
  // robot.move_distance(19.2);
  robot.move_distance(19.2);
  // robot.Rotation_move_imu(90);
  // robot.Rotation_move_imu(-90);
  // robot.rotate_angle(90);
  // robot.rotate_angle(-90);

  // while(millis()-prev_time<2000);
  // prev_time=millis();
  // robot.Rotation_move_imu(-90);
  // while(millis()-prev_time<2000);
  // prev_time=millis();

  //  Serial.println("---------------------------------------------------------------------");

  // solve();
}