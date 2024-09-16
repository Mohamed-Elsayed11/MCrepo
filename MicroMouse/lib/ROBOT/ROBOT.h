#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <WiFiNINA.h>
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

#define front_IR A2
#define right_IR 13
#define left_IR 12

class ROBOT
{
public:
    DC_MOTOR right_motor = DC_MOTOR(input1_1, input2_1, enable_1, encoder1_1, encoder2_1);
    DC_MOTOR left_motor = DC_MOTOR(input1_2, input2_2, enable_2, encoder1_2, encoder2_2);

    PID right_velocity_pid = PID(0.565, 0.0, 0.0, 200); // left
    PID left_velocity_pid = PID(0.38, 0.0, 0.00, 200);  // Right

    PID right_pos_pid = PID(3, 0.015, 0.1, 2050);
    PID left_pos_pid = PID(3, 0.015, 0.0, 1900);

    PID imu_pid = PID(2.3, 0.00, 0.0, 100);
    PID forward_imu_pid = PID(3, 0.01, 0.0, 50);/*3 0.01 0*/

    PID encoder1_pid = PID(2.5, 0.003, 0.3, 255); // left
    PID encoder2_pid = PID(2.5, 0.008, 0.1, 255); // right

    IMU2040 imu = IMU2040();
    double wheelSeparation, wheelDiameter;
    int PPR; // pulse per revolution
    unsigned long stopping_time = 2500, prev_time = 0, last_update_time = 0;
    int right_pos_setpoint = 0, left_pos_setpoint = 0, angle_setpoint = 0;
    bool touchBehindWall = true;
    long unsigned prev_back_time = 0;

public:
    ROBOT(double wheelSeparation, double wheelDiameter, int PPR)
        : wheelSeparation(wheelSeparation), wheelDiameter(wheelDiameter), PPR(PPR) {}

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
        else
        {
            imu.calulations();
            angle_setpoint = imu.get_Yaw_angle();
            Serial.println("IMU initialized");
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
        forward_imu_pid.setSetpoint(angle_setpoint);
        prev_time = millis();
        while (millis() - prev_time < stopping_time)
        {
            unsigned long current_time = millis();
            if (current_time - last_update_time >= 30)
            {
                right_motor.update_velocity_1(current_time);
                left_motor.update_velocity_2(current_time);

                right_pos_pid.setFeedback(right_motor.get_pos_feedback_1());
                left_pos_pid.setFeedback(left_motor.get_pos_feedback_2());

                imu.calulations();
                forward_imu_pid.setFeedback(imu.get_Yaw_angle());

                right_velocity_pid.setSetpoint(right_pos_pid.compute());
                left_velocity_pid.setSetpoint(left_pos_pid.compute());
                right_velocity_pid.setFeedback(right_motor.get_velocity_1());
                left_velocity_pid.setFeedback(left_motor.get_velocity_2());
                int speed1 = right_velocity_pid.compute();
                int speed2 = left_velocity_pid.compute();
                int speed3 = forward_imu_pid.compute();
                forward_imu_pid.direction > 0 ? speed3 = speed3 : speed3 = -speed3;
                speed1 -= speed3; speed2 += speed3;
                right_pos_pid.direction > 0 ? right_motor.forward(speed1) : right_motor.backward(speed1);
                left_pos_pid.direction > 0 ? left_motor.forward(speed2) : left_motor.backward(speed2);
                // Serial.print("right motor: ");
                // Serial.print(left_motor.get_pos_feedback_2());
                // Serial.print("\t\t");
                // Serial.print("left motor: ");
                // Serial.println(right_motor.get_pos_feedback_1());
                last_update_time = current_time;
            }
            // if (abs(right_pos_pid.getError()) < 30 && abs(left_pos_pid.getError()) < 30)
            // {
            //     break;
            // }
        }
        // right_motor.reset_pos_1();
        // left_motor.reset_pos_2();
        this->stop();
    }

    void rotate_angle(double angle)
    {
        // right_motor.reset_pos_1();
        // left_motor.reset_pos_2();
        double distance = (angle / 360) * 3.14 * wheelSeparation;
        double circumference = 3.14 * wheelDiameter;
        double rotations_no = distance / circumference;
        int pulses = rotations_no * PPR;
        right_pos_setpoint += pulses;
        left_pos_setpoint -= pulses;
        // encoder1_pid.setSetpoint(pulses);
        // encoder2_pid.setSetpoint(-pulses);
        encoder1_pid.setSetpoint(right_pos_setpoint);
        encoder2_pid.setSetpoint(left_pos_setpoint);
        prev_time = millis();
        while (millis() - prev_time < stopping_time)
        {
            imu.calulations();
            Serial.print("imu: ");
            Serial.println(imu.get_Yaw_angle());
            encoder1_pid.setFeedback(right_motor.get_pos_feedback_1());
            encoder2_pid.setFeedback(left_motor.get_pos_feedback_2());
            // Serial.print("left motor = ");
            // Serial.print(encoder1_pid.getFeedback());
            // Serial.print('\t');
            // Serial.print("right motor = ");
            // Serial.println(encoder2_pid.getFeedback());
            delay(5);
            int speed1 = encoder1_pid.compute();
            int speed2 = encoder2_pid.compute();
            encoder1_pid.direction > 0 ? right_motor.forward(speed1) : right_motor.backward(speed1);
            encoder2_pid.direction > 0 ? left_motor.forward(speed2) : left_motor.backward(speed2);
        }
        // right_pos_setpoint = 0;
        // left_pos_setpoint = 0;
        // right_motor.reset_pos_1();
        // left_motor.reset_pos_2();
    }

    void Rotation_move_imu(double angle)
    {
        angle_setpoint += angle;
        // angle_setpoint = angle;
        imu_pid.setSetpoint(angle_setpoint);
        prev_time = millis();
        while (millis() - prev_time < 6000)
        {
            imu.calulations();
            double yaw_angle = imu.get_Yaw_angle();
            imu_pid.setFeedback(yaw_angle);

            // Serial.print("angle_setpoint: ");
            // Serial.print(angle_setpoint);
            // Serial.print("Feedback: ");
            // Serial.println(yaw_angle);

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

            if (abs(imu_pid.getError()) < 3)
            {
                // imu.reset();
                // this->stop();
                break;
            }
        }
        // imu.reset();
        // right_motor.reset_pos_1();
        // left_motor.reset_pos_2();
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
        // delay(200);
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