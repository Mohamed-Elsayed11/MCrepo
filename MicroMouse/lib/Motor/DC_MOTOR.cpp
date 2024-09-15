#include "DC_MOTOR.h"
int DC_MOTOR::motor_num = 0;
DC_MOTOR* DC_MOTOR::motor1_ptr = nullptr;
DC_MOTOR* DC_MOTOR::motor2_ptr = nullptr;

DC_MOTOR::DC_MOTOR(int In1, int In2, int Enable, int EncA, int EncB)
    : in1(In1), in2(In2), EN(Enable), encA(EncA), encB(EncB), speed(150), pos_1(0)
    , pos_2(0), last_pos_1(0), last_pos_2(0), last_time_1(0), last_time_2(0)
    , velocity_1(0), velocity_2(2)
{
    ++motor_num;
    if (motor_num == 1) {
      motor1_ptr = this;
    }
    else if(motor_num == 2){
      motor2_ptr = this; 
    }
}

void DC_MOTOR::init() {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(EN, OUTPUT);
    pinMode(encA, INPUT_PULLUP);
    pinMode(encB, INPUT_PULLUP);

    if (this == motor1_ptr) {
        attachInterrupt(digitalPinToInterrupt(encA), DC_MOTOR::encoder_callback1, RISING);
    }
    else if (this == motor2_ptr) {
        attachInterrupt(digitalPinToInterrupt(encA), DC_MOTOR::encoder_callback2, RISING);
    }
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

void DC_MOTOR::encoder_callback1() {
    motor1_ptr->handle_encoder_1();
}

void DC_MOTOR::encoder_callback2() {
    motor2_ptr->handle_encoder_2();
}

void DC_MOTOR::handle_encoder_1() {
    int Encoder_B = digitalRead(encB);
    if (Encoder_B > 0) {
        pos_1++;
    } else {
        pos_1--;
    }
}
void DC_MOTOR::handle_encoder_2() {
    int Encoder_B = digitalRead(encB);
    if (Encoder_B > 0) {
        pos_2--;
    } else {
        pos_2++;
    }
}

int DC_MOTOR::get_pos_feedback_1() {
    return pos_1;
}
int DC_MOTOR::get_pos_feedback_2() {
    return pos_2;
}
void DC_MOTOR::reset_pos_1() {
    pos_1 = 0;
}
void DC_MOTOR::reset_pos_2() {
    pos_2 = 0;
}

void DC_MOTOR::update_velocity_1(unsigned long current_time) {
    unsigned long delta_time = current_time - last_time_1;
    int delta_pos = pos_1 - last_pos_1;

    if (delta_time > 0) {
        velocity_1 = (delta_pos * 1000.0) / delta_time;
    }
    Filtered_velocity_1=(LPF*velocity_1)+((1-LPF)*Filtered_velocity_1);
    last_pos_1 = pos_1;
    last_time_1 = current_time;
}

void DC_MOTOR::update_velocity_2(unsigned long current_time) {
    unsigned long delta_time = current_time - last_time_2;
    int delta_pos = pos_2 - last_pos_2;

    if (delta_time > 0) {
        velocity_2 = (delta_pos * 1000.0) / delta_time;
    }
    Filtered_velocity_2=(LPF*velocity_2)+((1-LPF)*Filtered_velocity_2);
    last_pos_2 = pos_2;
    last_time_2 = current_time;
}

int DC_MOTOR::get_velocity_1(){
    return Filtered_velocity_1;
}

int DC_MOTOR::get_velocity_2(){
    return Filtered_velocity_2;
}