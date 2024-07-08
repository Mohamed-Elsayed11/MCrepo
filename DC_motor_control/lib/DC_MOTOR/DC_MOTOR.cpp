#include "DC_MOTOR.h"

DC_MOTOR::DC_MOTOR(int In1, int In2, int Enable, int ENC1, int ENC2) {
  in1 = In1;
  in2 = In2;
  EN = Enable;
  enc1 = ENC1;
  enc2 = ENC2;
  speed = 255;
}

void DC_MOTOR::init() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(enc1, INPUT);
  pinMode(enc2, INPUT);
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
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(EN, Speed);
}

void DC_MOTOR::backward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(EN, speed);
}

void DC_MOTOR::backward(int Speed) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(EN, Speed);
}

void DC_MOTOR::stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(EN, 0);
}
