#include <Arduino.h>
#include "ROBOT.h"

ROBOT robot = ROBOT(10, 6, 1000);

void setup()
{
    Serial.begin(9600);
    robot.init();
}

void loop()
{
    robot.move_distance(6.5*3.14);
    robot.move_distance(-30);
    robot.rotate_angle(90);
    robot.rotate_angle(-90);
    // robot.Rotaion_move_imu(90);
}