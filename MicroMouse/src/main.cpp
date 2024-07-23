#include <Arduino.h>
#include "DC_MOTOR.h"
#include "PID.h"

#define input1 4
#define input2 5
#define enable 6
#define encoder1 2
#define encoder2 3

DC_MOTOR motor1 = DC_MOTOR(input1, input2, enable, encoder1, encoder2);
PID pid = PID(0.45, 0.0025, 0.001);

void setup() {
    Serial.begin(9600);
    motor1.init();
    pid.setSetpoint(2000);
}

void loop() {
    int feedback = motor1.get_pos_feedback();
    Serial.println(feedback);
    int speed = pid.compute(feedback);
    pid.direction > 0 ? motor1.forward(speed) : motor1.backward(speed);
}