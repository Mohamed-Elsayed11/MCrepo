#include <Arduino.h>
#include "ROBOT.h"

ROBOT robot = ROBOT(10, 6, 500);

void setup()
{
    Serial.begin(9600);
    robot.init();
}

void loop()
{
    robot.move_distance(30);
    robot.move_distance(-30);
    robot.rotate_angle(90);
    robot.rotate_angle(-90);
}