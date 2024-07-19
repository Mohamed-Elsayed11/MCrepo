#include <Arduino.h>
#include "pid.h"

PID::PID(float kp, float ki, float kd)

{
    KP = kp;
    ki = ki;
    kd = kd;
}

void PID::error_estimation(void)
{
    now = millis();
    dt = ((float)(now - last_time) / 1000);
    last_time = now;
    // error = set_point - pos;
    controlled_signal = pid(error);
    output = fabs(controlled_signal);
    if (output > 255)
    {
        output = 255;
    }
    if (controlled_signal < 0)
    {
        // dir = 1;
    }
    else
    {
        // dir = 0;
    }
}
int PID::pid(int error)
{
    integral += error * dt;
    derivative = (error - pervious_error) / dt;
    pervious_error = error;
    return (KP * error + KI * integral + KD * derivative);
}
