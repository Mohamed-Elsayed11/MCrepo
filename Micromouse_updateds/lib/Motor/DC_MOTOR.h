#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <Arduino.h>

#define MAX_MOTORS 2

class DC_MOTOR {
public:
    DC_MOTOR(int In1, int In2, int Enable, int EncA, int EncB);
    void init();
    void setSpeed(int Speed);
    void forward();
    void forward(int Speed);
    void backward();
    void backward(int Speed);
    void stop();
    int get_pos_feedback();

private:
    static int motor_num;
    static DC_MOTOR* motor1_ptr;
    static DC_MOTOR* motor2_ptr;
    int in1, in2, EN, encA, encB;
    int speed;
    volatile long pos;

    static void encoder_callback1();
    static void encoder_callback2();
    void handle_encoder();
};

#endif
