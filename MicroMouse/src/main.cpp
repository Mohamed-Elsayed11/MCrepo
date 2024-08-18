#include <Arduino.h>
// #include "ROBOT.h"
#include "FloodFill.h"

// ROBOT robot = ROBOT(11, 6.5, 970);

void setup()
{
   
  Serial.begin(9600);
  initialize();
  // robot.init();
  // while(!Serial){
  //   ;
  // }
  // robot.move_distance(19.2);
}

void loop()
{
  solve();

  // robot.move_distance(19.2);

  // robot.rotate_angle(90);
  
}