#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "PID.h"
#include "DC_MOTOR.h"
#include "IMU.h"

#define input1_1    6
#define input2_1    5
#define enable_1    7
#define encoder1_1  9
#define encoder2_1  8

#define input1_2 4
#define input2_2 3
#define enable_2 2
#define encoder1_2 11
#define encoder2_2 10

class ROBOT
{
private:
    PID encoder1_pid = PID(0.65, 0.000,0.000);
    PID encoder2_pid = PID(0.85,0,0);
    PID imu_pid = PID(2.5, 0.025, 0.001);
    DC_MOTOR right_motor = DC_MOTOR(input1_1, input2_1, enable_1, encoder1_1, encoder2_1);
    DC_MOTOR left_motor =  DC_MOTOR(input1_2, input2_2, enable_2, encoder1_2, encoder2_2);
    IMU2040 imu = IMU2040();
    double wheelSeparation, wheelDiameter, encoder1_prev_error, encoder2_prev_error;
    double imu_prev_error;
    int PPR; // pulse per revolution
    unsigned long stopping_time, prev_time;
    unsigned long imu_stopping_time, imu_prev_time;

public:
    ROBOT(double wheelSeparation, double wheelDiameter, int PPR)
        : wheelSeparation(wheelSeparation), wheelDiameter(wheelDiameter),
          PPR(PPR), stopping_time(600), prev_time(0), encoder1_prev_error(0),
          encoder2_prev_error(0), imu_prev_error(0), imu_stopping_time(1000), imu_prev_time(0) {}

    void init()
    {
        /*  while (!Serial)
              ;
          if (!imu.init())
          {
              Serial.println("IMU initialization failed!");
              while (1)
                  ;
          }*/
          right_motor.init();
          left_motor.init();
       
    }

    void move_distance(double distance)
    {
        right_motor.reset_pos_1();
        left_motor.reset_pos_2();
        double circumference = 3.14 * wheelDiameter;
        double rotations_no = distance / circumference;
        int pulses = rotations_no * PPR;
        encoder1_pid.setSetpoint(pulses);
        encoder2_pid.setSetpoint(pulses);
        while (!ROBOT::isStopped())
        {
            encoder1_pid.setFeedback(right_motor.get_pos_feedback_1());
            encoder2_pid.setFeedback(left_motor.get_pos_feedback_2());
            delay(10);
            int speed1 = encoder1_pid.compute();
            int speed2 = encoder2_pid.compute();
            encoder1_pid.direction > 0 ? right_motor.forward(speed1) : right_motor.backward(speed1);
            encoder2_pid.direction > 0 ? left_motor.forward(speed2) : left_motor.backward(speed2);
        }
    }

    void rotate_angle(double angle)
    {
        right_motor.reset_pos_1();
        left_motor.reset_pos_2();
        double distance = (angle / 360) * 3.14 * wheelSeparation;
        double circumference = 3.14 * wheelDiameter;
        double rotations_no = distance / circumference;
        int pulses = rotations_no * PPR;
        encoder1_pid.setSetpoint(pulses);
        encoder2_pid.setSetpoint(-pulses);
        while (!ROBOT::isStopped())
        {
            encoder1_pid.setFeedback(right_motor.get_pos_feedback_1());
            encoder2_pid.setFeedback(left_motor.get_pos_feedback_2());
            delay(10);
            int speed1 = encoder1_pid.compute();
            int speed2 = encoder2_pid.compute();
            encoder1_pid.direction > 0 ? right_motor.forward(speed1) : right_motor.backward(speed1);
            encoder2_pid.direction > 0 ? left_motor.forward(speed2) : left_motor.backward(speed2);
        }
    }

    void Rotaion_move_imu(double angle)
    {

        imu_pid.setSetpoint(angle);
        while (!ROBOT::isStopped_imu())
        {
            imu.calulations();
            imu_pid.setFeedback(imu.get_Yaw_angle());
            Serial.print("feedback: ");
            Serial.println(imu.get_Yaw_angle());
            int speed = imu_pid.compute();
            imu_pid.direction > 0 ? right_motor.forward(speed), left_motor.backward(speed) : left_motor.forward(speed), right_motor.backward(speed);
        }
    }

    bool isStopped()
    {
        if (millis() - prev_time >= stopping_time)
        {
            if (encoder1_pid.getError() == encoder1_prev_error &&
                encoder2_pid.getError() == encoder2_prev_error)
            {
                return true;
            }
            encoder1_prev_error = encoder1_pid.getError();
            encoder2_prev_error = encoder2_pid.getError();
            prev_time = millis();
        }
        return false;
    }

    bool isStopped_imu()
    {
        if (millis() - imu_prev_time >= imu_stopping_time)
        {
            if (imu_pid.getError() == imu_prev_error)
            {
                return true;
            }
            imu_prev_time = millis();
        }
        return false;
    }
};

#endif
