#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
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

#define front_IR A5
#define right_IR 12
#define left_IR 13

class ROBOT
{
public:
    PID encoder1_pid = PID(2, 0.003, 0.3, 255); // left
    PID encoder2_pid = PID(2.5, 0.008, 0.1, 255); // right
    PID imu_pid = PID(3,0.02, 0.0, 255);
    
    DC_MOTOR right_motor = DC_MOTOR(input1_1, input2_1, enable_1, encoder1_1, encoder2_1);
    DC_MOTOR left_motor =  DC_MOTOR(input1_2, input2_2, enable_2, encoder1_2, encoder2_2);

    PID right_velocity_pid = PID(0.565, 0.0, 0.0, 255);
    PID left_velocity_pid = PID(0.38, 0.0, 0.0, 255);
    PID right_pos_pid = PID(3.2, 0.0, 0.0, 2000);
    PID left_pos_pid = PID(3, 0.0, 0.0, 1870);

    IMU2040 imu = IMU2040();
    double wheelSeparation, wheelDiameter, encoder1_prev_error, encoder2_prev_error;
    double imu_prev_error;
    int PPR; // pulse per revolution
    unsigned long stopping_time, prev_time, last_update_time = 0;
    unsigned long imu_stopping_time, imu_prev_time;
    int repeat = 0, right_pos_setpoint = 0, left_pos_setpoint = 0;

public:
    ROBOT(double wheelSeparation, double wheelDiameter, int PPR)
        : wheelSeparation(wheelSeparation), wheelDiameter(wheelDiameter),
          PPR(PPR), stopping_time(3000), prev_time(0), encoder1_prev_error(0),
          encoder2_prev_error(0), imu_prev_error(0), imu_stopping_time(1000), imu_prev_time(0) {}

    void init()
    {
        // while (!Serial)
        //     ;

        if (!imu.init())
        {
            Serial.println("IMU initialization failed!");
            while (1)
                ;
        }

        right_motor.init();
        left_motor.init();
        IR_init();
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
        prev_time = millis();
        while (millis() - prev_time < stopping_time)
        {
            unsigned long current_time = millis();
            if (current_time - last_update_time >= 30) {
                right_motor.update_velocity_1(current_time);
                left_motor.update_velocity_2(current_time);
                right_pos_pid.setFeedback(right_motor.get_pos_feedback_1());
                left_pos_pid.setFeedback(left_motor.get_pos_feedback_2());
                right_velocity_pid.setSetpoint(right_pos_pid.compute());
                left_velocity_pid.setSetpoint(left_pos_pid.compute());
                right_velocity_pid.setFeedback(right_motor.get_velocity_1());
                left_velocity_pid.setFeedback(left_motor.get_velocity_2());
                int speed1 = right_velocity_pid.compute();
                int speed2 = left_velocity_pid.compute();
                right_pos_pid.direction > 0 ? 
                right_motor.forward(speed1) : right_motor.backward(speed1);
                left_pos_pid.direction > 0 ? 
                left_motor.forward(speed2) : left_motor.backward(speed2); 
                Serial.print("right motor: ");
                Serial.print(left_motor.get_pos_feedback_2());
                Serial.print("\t\t");
                Serial.print("left right: ");
                Serial.println(right_motor.get_pos_feedback_1());
                last_update_time = current_time;
            }           
        }
        this->stop();
    }

    void rotate_angle(double angle)
    {
        double distance = (angle / 360) * 3.14 * wheelSeparation;
        double circumference = 3.14 * wheelDiameter;
        double rotations_no = distance / circumference;
        int pulses = rotations_no * PPR;
        right_pos_setpoint -= pulses;
        left_pos_setpoint += pulses;
        right_pos_pid.setSetpoint(right_pos_setpoint);
        left_pos_pid.setSetpoint(left_pos_setpoint);
        prev_time = millis();
        while (millis() - prev_time < stopping_time)
        {
            unsigned long current_time = millis();
            if (current_time - last_update_time >= 30) {
                right_motor.update_velocity_1(current_time);
                left_motor.update_velocity_2(current_time);
                right_pos_pid.setFeedback(right_motor.get_pos_feedback_1());
                left_pos_pid.setFeedback(left_motor.get_pos_feedback_2());
                right_velocity_pid.setSetpoint(right_pos_pid.compute());
                left_velocity_pid.setSetpoint(left_pos_pid.compute());
                right_velocity_pid.setFeedback(right_motor.get_velocity_1());
                left_velocity_pid.setFeedback(left_motor.get_velocity_2());
                int speed1 = right_velocity_pid.compute();
                int speed2 = left_velocity_pid.compute();
                right_pos_pid.direction > 0 ? 
                right_motor.forward(speed1) : right_motor.backward(speed1);
                left_pos_pid.direction > 0 ? 
                left_motor.forward(speed2) : left_motor.backward(speed2); 
                Serial.print("right motor: ");
                Serial.print(left_motor.get_pos_feedback_2());
                Serial.print("\t\t");
                Serial.print("left right: ");
                Serial.println(right_motor.get_pos_feedback_1());
                last_update_time = current_time;
            }           
        }
        this->stop();
    }

    void Rotaion_move_imu(double angle)
    {

        imu_pid.setSetpoint(angle);
        while (millis() - prev_time < stopping_time)
        {
            imu.calulations();
            imu_pid.setFeedback(imu.get_Yaw_angle());
            Serial.print("feedback: ");
            Serial.println(imu.get_Yaw_angle());
            int speed = imu_pid.compute();
            imu_pid.direction > 0 ? right_motor.forward(speed), left_motor.backward(speed) : left_motor.forward(speed), right_motor.backward(speed);
        }
    }

    void stop()
    {
        right_motor.stop();
        left_motor.stop();
    }

    void IR_init()
    {
        pinMode(front_IR, INPUT);
        pinMode(right_IR, INPUT);
        pinMode(left_IR, INPUT);
    }

    bool isFrontWall()
    {
        return !digitalRead(front_IR);
    }

    bool isRightWall()
    {
        return !digitalRead(right_IR);
    }

    bool isLeftWall()
    {
        return !digitalRead(left_IR);
    }
};

#endif