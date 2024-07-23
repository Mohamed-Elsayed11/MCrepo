#include "DC_MOTOR.h"

DC_MOTOR* DC_MOTOR::instance = nullptr;

DC_MOTOR::DC_MOTOR(int In1, int In2, int Enable, int ENC1, int ENC2)
{
  in1 = In1;
  in2 = In2;
  EN = Enable;
  enc1 = ENC1;
  enc2 = ENC2;
  speed = 255;
  pos = 0;
  instance = this;
}

void DC_MOTOR::init() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(enc1, INPUT_PULLUP);
  pinMode(enc2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1), enc1_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2), enc2_callback, CHANGE);
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

void DC_MOTOR::enc1_callback() {
  if (instance) {
    bool pinB = digitalRead(instance->enc2);
    bool pinA = digitalRead(instance->enc1);

    if (pinB == LOW) {
      if (pinA == HIGH) {
        instance->pos++;
      } else {
        instance->pos--;
      }
    } else {
      if (pinA == HIGH) {
        instance->pos--;
      } else {
        instance->pos++;
      }
    }
  }
}

void DC_MOTOR::enc2_callback() {
  if (instance) {
    bool pinB = digitalRead(instance->enc1);
    bool pinA = digitalRead(instance->enc2);

    if (pinA == LOW) {
      if (pinB == HIGH) {
        instance->pos--;
      } else {
        instance->pos++;
      }
    } else {
      if (pinB == HIGH) {
        instance->pos++;
      } else {
        instance->pos--;
      }
    }
  }
}

int DC_MOTOR::get_pos_feedback() {
  return pos;
}