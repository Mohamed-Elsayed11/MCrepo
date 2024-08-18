#include <Arduino.h>
#include "ROBOT.h"

ROBOT robot = ROBOT(11, 6.5, 970);

void setup()
{
   
  Serial.begin(9600);
  robot.init();
  // while(!Serial){
  //   ;
  // }
  // robot.move_distance(19.2);
}

void loop()
{
  robot.move_distance(19.2);

  robot.rotate_angle(90);
  
}