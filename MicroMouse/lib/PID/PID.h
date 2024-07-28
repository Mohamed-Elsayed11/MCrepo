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

    public:
        PID(double kp, double ki, double kd);
        void setSetpoint(double setpoint);
        double compute(double feedback);
        int direction;
};

#endif
