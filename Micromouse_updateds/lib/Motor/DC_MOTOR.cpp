#include "DC_MOTOR.h"

DC_MOTOR* DC_MOTOR::instance = nullptr;

DC_MOTOR::DC_MOTOR(int In1, int In2, int Enable, int EncA, int EncB): in1(In1), in2(In2), EN(Enable), encA(EncA), encB(EncB), speed(150), pos(0) 
{
  if (instance == nullptr) {
    instance = this;
  }
}

void DC_MOTOR::init() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encA), encA_callback, RISING);

}
 
void DC_MOTOR::setSpeed(int Speed) {
  speed = Speed;
}

void DC_MOTOR::forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(EN, speed);
}

void DC_MOTOR::forward(int Speed) {
  setSpeed(Speed);
  forward();
}

void DC_MOTOR::backward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(EN, speed);
}

void DC_MOTOR::backward(int Speed) {
  setSpeed(Speed);
  backward();
}

void DC_MOTOR::stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(EN, 0);
}

void DC_MOTOR::encA_callback() {
  if (instance) {
    int Encoder_B = digitalRead(instance->encB);
    
    if (Encoder_B > 0) {
      instance->pos++;
    } else {
      instance->pos--;
    }
  }
}

int DC_MOTOR::get_pos_feedback() {
  return pos;
}
