#include <Arduino.h>
#include "ROBOT.h"
// #include "FloodFill.h"

 ROBOT robot = ROBOT(9.7, 6.5, 970);

void setup()
{
  // while(!Serial){
  //   ;
  // }

  Serial.begin(9600);
  // initialize();
  robot.init();
  
  for(int i = 0; i < 4; i++){
    robot.move_distance(19.2);
  }
   
  // robot.move_distance(19.2);
  // robot.rotate_angle(90); 
  
}

 

void loop()
{
  // solve();
  // robot.move_distance(19.2);
  // robot.Rotaion_move_imu(90);
  // robot.rotate_angle(90);  
}