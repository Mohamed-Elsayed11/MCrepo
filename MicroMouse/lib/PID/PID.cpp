#include "PID.h"

PID::PID(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->setpoint = 0;
    this->prevError = 0;
    this->integral = 0;
    this->lastTime = 0;
    this->direction = 0;
}

void PID::setSetpoint(double setpoint) {
    this->setpoint = setpoint;
}

double PID::compute(double feedback) {
    double error = setpoint - feedback;

    unsigned long currentTime = millis();

    double deltaTime = (currentTime - lastTime) / 1000.0;

    integral += error * deltaTime;

    double derivative = (error - prevError) / deltaTime;

    double output = kp * error + ki * integral + kd * derivative;

    output > 0 ? this->direction = 1 : this->direction = -1;

    output = abs(output);

    output = min(output, 255);

    prevError = error;
    lastTime = currentTime;

    return output;
}
