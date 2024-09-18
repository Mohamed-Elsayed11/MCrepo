#include "PID.h"

PID::PID(float kp, float ki, float kd, int max_output, int windup_limit)
        :kp(kp), ki(ki), kd(kd), max_output(max_output), windup_limit(windup_limit), 
        setpoint(0), prevError(0), integral(0), lastTime(0), direction(0), feedback(0){}

void PID::setSetpoint(float setpoint) {
    this->setpoint = setpoint;
}

void PID::setFeedback(int feedback) {
    this->feedback = feedback;
}

int PID::getFeedback() {
    return this->feedback;
}

float PID::getError(){
    return setpoint - feedback;
}

float PID::compute() {
    float error = setpoint - feedback;
    prevError = error;
    unsigned long currentTime = millis();

    float deltaTime = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;
    integral += error * deltaTime;

    if(integral > windup_limit){
        integral = windup_limit;
    }

    float derivative = (error - prevError) / deltaTime;

    float output = kp * error + ki * integral + kd * derivative;

    output > 0 ? this->direction = 1 : this->direction = -1;

    output = abs(output);

    output = min(output, max_output);

    return output;
}
