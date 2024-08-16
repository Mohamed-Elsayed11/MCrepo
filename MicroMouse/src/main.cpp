#include <Arduino.h>
#include "ROBOT.h"


ROBOT robot = ROBOT(11, 6.5, 970);

void setup()
{
   
    Serial.begin(9600);
    robot.init();
    
}

void loop()
{
            robot.move_distance(19.2);
          
          
          // robot.move_distance(-30);
            robot.rotate_angle(90);
           
            robot.move_distance(19.2);
           
           robot.rotate_angle(90);
          
           robot.move_distance(19.2);
           
         //  robot.rotate_angle(-90);
          // delay(50);


           
          
           delay(100);
           //robot.Rotaion_move_imu(90);
}