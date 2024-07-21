#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <Arduino.h>

class DC_MOTOR {
  private:
    int in1, in2, EN, enc1, enc2;
    int speed;
    volatile static int pos;
    static DC_MOTOR* instance;

  public:
    DC_MOTOR(int In1, int In2, int Enable, int ENC1, int ENC2);
    void init();
    void setSpeed(int Speed);
    void forward();
    void forward(int Speed);
    void backward();
    void backward(int Speed);
    void stop();
    int get_pos_feedback();
    static void enc1_callback();
    static void enc2_callback();
};

#endif
