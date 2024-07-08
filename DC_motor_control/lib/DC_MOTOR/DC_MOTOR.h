#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <Arduino.h>

class DC_MOTOR {
  private:
    int in1;
    int in2;
    int EN;
    int enc1;
    int enc2;
    int speed;

  public:
    DC_MOTOR(int In1, int In2, int Enable, int ENC1, int ENC2);
    void init();
    void setSpeed(int Speed);
    void forward();
    void forward(int Speed);
    void backward();
    void backward(int Speed);
    void stop();
};

#endif
