#include <Arduino.h>
#include "ROBOT.h"
// #include "FloodFill.h"
// #include <DC_MOTOR.h>

ROBOT robot = ROBOT(9.7, 6.5, 970);

unsigned long last_update_time = 0;

void setup()
{
  // while(!Serial){
  //   ;
  // }

  Serial.begin(9600);
  robot.right_motor.init();
  robot.left_motor.init();
  // initialize();
  // robot.init();
  
  // for(int i = 0; i < 3; i++){
  //   robot.move_distance(19.2);
  // }
   
  // robot.move_distance(19.2);
  // robot.rotate_angle(90); 
  
  robot.right_motor.forward(100);
  robot.left_motor.forward(100);
  unsigned long last_update_time = millis();
}

 

void loop()
{
  unsigned long current_time = millis();

  if (current_time - last_update_time >= 100) {
      robot.right_motor.update_velocity_1(current_time);
      robot.left_motor.update_velocity_2(current_time);
      Serial.print("left motor: ");
      Serial.print(robot.left_motor.get_velocity_2());
      Serial.print('\t');
      Serial.print("right right: ");
      Serial.println(robot.right_motor.get_velocity_1());
      last_update_time = current_time;
  }
  // solve();
  // robot.move_distance(19.2);
  // robot.Rotaion_move_imu(90);
  // robot.rotate_angle(90);  
}