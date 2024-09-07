#include <Arduino.h>
#include "ROBOT.h"
// #include "FloodFill.h"

ROBOT robot = ROBOT(9.5, 6.5, 970);

void setup()
{
  // while(!Serial){
  //   ;
  // }

  Serial.begin(9600);

  // initialize();

  robot.init();
}

void loop()
{
  for(int i = 0; i < 6; i++){
    robot.move_distance(19.2);
  }

  for(int i = 0; i < 2; i++){
    robot.rotate_angle(90);
  }

  // solve();
}