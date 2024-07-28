#include "PID.h"

PID::PID(double kp, double ki, double kd)
        :kp(kp), ki(ki), kd(kd), setpoint(0), prevError(0), integral(0), 
        lastTime(0), direction(0), feedback(0){}

void PID::setSetpoint(double setpoint) {
    this->setpoint = setpoint;
}

void PID::setFeedback(int feedback) {
    this->feedback = feedback;
}

int PID::getFeedback() {
    return this->feedback;
}

double PID::getError(){
    return setpoint - feedback;
}

double PID::compute() {
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
