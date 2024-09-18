#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
    private:
        float kp;      
        float ki;      
        float kd;      
        float setpoint; 
        float prevError; 
        float integral;  
        unsigned long lastTime; 
        int feedback, max_output, windup_limit;

    public:
        PID(float kp, float ki, float kd, int max_output, int windup_limit);
        void setSetpoint(float setpoint);
        void setFeedback(int feedback);
        int getFeedback();
        float getError();
        float compute();
        int direction;
};

#endif
