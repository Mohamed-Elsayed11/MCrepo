#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
    private:
        double kp;      
        double ki;      
        double kd;      
        double setpoint; 
        double prevError; 
        double integral;  
        unsigned long lastTime; 
        int feedback, max_output;

    public:
        PID(double kp, double ki, double kd, int max_output);
        void setSetpoint(double setpoint);
        void setFeedback(int feedback);
        int getFeedback();
        double getError();
        double compute();
        int direction;
};

#endif
