#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <WiFiNINA.h>
#include <Adafruit_VL6180X.h>
#include "PID.h"
#include "DC_MOTOR.h"
#include "IMU.h"

#define input1_1 6
#define input2_1 5
#define enable_1 7
#define encoder1_1 9
#define encoder2_1 8

#define input1_2 4
#define input2_2 3
#define enable_2 2
#define encoder1_2 11
#define encoder2_2 10

MbedI2C I2C1(A0, A1);
#define TOF1_ADDRESS 0x30
#define TOF2_ADDRESS 0x31
#define right_SHUT A3
#define left_SHUT A2

#define front_IR 13

class ROBOT
{
private:
    DC_MOTOR right_motor = DC_MOTOR(input1_1, input2_1, enable_1, encoder1_1, encoder2_1);
    DC_MOTOR left_motor = DC_MOTOR(input1_2, input2_2, enable_2, encoder1_2, encoder2_2);

    PID right_velocity_pid = PID(0.565, 0.0, 0.0, 200, 20); // left
    PID left_velocity_pid =  PID(0.38, 0.0, 0.00, 200, 20);  // Right

    PID right_pos_pid = PID(2.7, 0.008,  0.1,  2000,  20);
    PID left_pos_pid =  PID(2.7, 0.008,  0.1,  2000,  20);

    PID imu_pid = PID(2.5, 0.001, 0.0, 85, 20);
    PID TOF_pid = PID(0.8, 0, 0, 50, 20);

    IMU2040 imu = IMU2040();

    Adafruit_VL6180X right_TOF = Adafruit_VL6180X();
    Adafruit_VL6180X left_TOF = Adafruit_VL6180X();

    double wheelSeparation, wheelDiameter;
    int PPR; // pulse per revolution
    unsigned long stopping_time = 2500, prev_time = 0, last_update_time = 0;
    int right_pos_setpoint = 0, left_pos_setpoint = 0, angle_setpoint = 0, TOF_error = 0;
    long unsigned prev_back_time = 0, last_TOF_update;
    bool TOF_correction = false;

public:
    ROBOT(double wheelSeparation, double wheelDiameter, int PPR)
        : wheelSeparation(wheelSeparation), wheelDiameter(wheelDiameter), PPR(PPR) {}

    bool touchBehindWall = true;

    void init()
    {
        right_motor.init();
        left_motor.init();
        pinMode(front_IR, INPUT);
        pinMode(right_SHUT, OUTPUT);
        pinMode(left_SHUT, OUTPUT);
        digitalWrite(right_SHUT, HIGH);
        digitalWrite(left_SHUT, LOW);
        right_TOF.begin(&I2C1);
        right_TOF.setAddress(TOF1_ADDRESS);
        delay(10);
        digitalWrite(left_SHUT, HIGH);
        delay(10);
        left_TOF.begin(&I2C1);
        left_TOF.setAddress(TOF2_ADDRESS);
        imu.init();
        imu.calulations();
        angle_setpoint = imu.get_Yaw_angle();
    }

    void move_distance(double distance)
    {
        double circumference = 3.14 * wheelDiameter;
        double rotations_no = distance / circumference;
        int pulses = rotations_no * PPR;
        right_pos_setpoint += pulses;
        left_pos_setpoint += pulses;
        right_pos_pid.setSetpoint(right_pos_setpoint);
        left_pos_pid.setSetpoint(left_pos_setpoint);
        TOF_pid.setSetpoint(0);
        prev_time = millis();
        while (millis() - prev_time < stopping_time)
        {
            if(millis() - last_TOF_update >= 100)
            {
                int right_distance = right_TOF.readRange();
                int left_distance = left_TOF.readRange();
                right_TOF.readRange() < 100 && left_TOF.readRange() < 100 ? TOF_correction = true : TOF_correction = false;
                TOF_error = right_distance - left_distance;
                last_TOF_update = millis();
            }
            unsigned long current_time = millis();
            if (current_time - last_update_time >= 30)
            {
                right_pos_pid.setFeedback(right_motor.get_pos_feedback_1());
                left_pos_pid.setFeedback(left_motor.get_pos_feedback_2());
                right_velocity_pid.setSetpoint(right_pos_pid.compute());
                left_velocity_pid.setSetpoint(left_pos_pid.compute());
                right_velocity_pid.setFeedback(right_motor.get_velocity_1());
                left_velocity_pid.setFeedback(left_motor.get_velocity_2());
                int speed1 = right_velocity_pid.compute();
                int speed2 = left_velocity_pid.compute();
                if(TOF_correction)
                {
                    TOF_pid.setFeedback(TOF_error);
                    int speed3 = TOF_pid.compute();
                    Serial.println(speed3);
                    TOF_pid.direction > 0 ? speed3 = speed3 : speed3 = -speed3;
                    speed1 -= speed3; speed2 += speed3;
                }
                right_pos_pid.getError() < 50 ? speed1 = 0 : speed1 = speed1;
                left_pos_pid.getError() < 50 ? speed2 = 0 : speed2 = speed2;
                right_pos_pid.direction > 0 ? right_motor.forward(speed1) : right_motor.backward(speed1);
                left_pos_pid.direction > 0 ? left_motor.forward(speed2) : left_motor.backward(speed2);
                last_update_time = current_time;
            }
            if (abs(right_pos_pid.getError()) < 50 && abs(left_pos_pid.getError()) < 50)
            {
                break;
            }
        }
        this->stop();
    }

    void Rotation_move_imu(double angle)
    {
        angle_setpoint += angle;
        imu_pid.setSetpoint(angle_setpoint);
        prev_time = millis();
        Serial.print("front: ");
        Serial.print(this->isFrontWall());
        Serial.print("right: ");
        Serial.print(this->isRightWall());
        Serial.print("left: ");
        Serial.println(this->isLeftWall());
        while (millis() - prev_time < 4000)
        {
            imu.calulations();
            double yaw_angle = imu.get_Yaw_angle();
            imu_pid.setFeedback(yaw_angle);
            int speed = imu_pid.compute();

            if (imu_pid.direction > 0)
            {
                left_motor.forward(speed);
                right_motor.backward(speed);
            }
            else
            {
                right_motor.forward(speed);
                left_motor.backward(speed);
            }

            if (abs(imu_pid.getError()) < 1)
            {
                break;
            }
        }
        right_pos_setpoint = right_motor.get_pos_feedback_1();
        left_pos_setpoint = left_motor.get_pos_feedback_2();
        this->stop();
    }

    void stop()
    {
        right_motor.stop();
        left_motor.stop();
    }

    void move_backward()
    {
        prev_back_time = millis();
        while(millis() - prev_back_time < 400){
            right_motor.backward(200);
            left_motor.backward(200);
        }
        right_pos_setpoint = 0;
        left_pos_setpoint = 0;
        right_motor.reset_pos_1();
        left_motor.reset_pos_2();
        this->stop();
    }

    bool isFrontWall()
    {
        return !digitalRead(front_IR);
    }

    bool isRightWall()
    {
        return right_TOF.readRange() < 75;
    }

    bool isLeftWall()
    {
        return left_TOF.readRange() < 75;
    }
};

#endif