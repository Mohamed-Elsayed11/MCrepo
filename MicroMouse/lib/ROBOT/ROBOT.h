#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "PID.h"
#include "DC_MOTOR.h"
// #include "IMU.h"

#define input1_1 7
#define input2_1 5
#define enable_1 6
#define encoder1_1 2
#define encoder2_1 4

#define input1_2 9
#define input2_2 10
#define enable_2 11
#define encoder1_2 3
#define encoder2_2 8

class ROBOT{
    private:
        PID encoder1_pid = PID(0.55, 0.0025, 0.001);
        PID encoder2_pid = PID(0.55, 0.0025, 0.001);
        PID imu_pid = PID(2.5, 0.0025, 0.001);
        DC_MOTOR right_motor = DC_MOTOR(input1_1, input2_1, enable_1, encoder1_1, encoder2_1);
        DC_MOTOR left_motor = DC_MOTOR(input1_2, input2_2, enable_2, encoder1_2, encoder2_2);
        double wheelSeparation, wheelDiameter, encoder1_prev_error, encoder2_prev_error;
        int PPR;    //pulse per revolution
        unsigned long stopping_time, prev_time;

    public:
        ROBOT(double wheelSeparation, double wheelDiameter, int PPR)
                :wheelSeparation(wheelSeparation), wheelDiameter(wheelDiameter), 
                PPR(PPR), stopping_time(1000), prev_time(0), encoder1_prev_error(0), encoder2_prev_error(0){}
        
        void init(){
            left_motor.init();
            right_motor.init();
        }

        void move_distance(double distance){
            right_motor.reset_pos();
            left_motor.reset_pos();
            double circumference = 3.14 * wheelDiameter;
            double rotations_no = distance / circumference;
            int pulses = rotations_no * PPR;
            encoder1_pid.setSetpoint(pulses);
            encoder2_pid.setSetpoint(pulses);
            while(!ROBOT::isStopped()){
                encoder1_pid.setFeedback(right_motor.get_pos_feedback());
                encoder2_pid.setFeedback(left_motor.get_pos_feedback());
                Serial.print("feedback1: ");
                Serial.print(encoder1_pid.getFeedback());
                Serial.print("\t");
                Serial.print("feedback2: ");
                Serial.print(encoder2_pid.getFeedback());
                Serial.print("\n");
                int speed1 = encoder1_pid.compute();
                int speed2 = encoder2_pid.compute();
                encoder1_pid.direction > 0 ? right_motor.forward(speed1) : right_motor.backward(speed1);
                encoder2_pid.direction > 0 ? left_motor.forward(speed2) : left_motor.backward(speed2);
            }
        }

        void rotate_angle(double angle){
            right_motor.reset_pos();
            left_motor.reset_pos();
            double distance = (angle/360) * 3.14 * wheelSeparation;
            double circumference = 3.14 * wheelDiameter;
            double rotations_no = distance / circumference;
            int pulses = rotations_no * PPR;
            encoder1_pid.setSetpoint(pulses);
            encoder2_pid.setSetpoint(-pulses);
            while(!ROBOT::isStopped()){
                encoder1_pid.setFeedback(right_motor.get_pos_feedback());
                encoder2_pid.setFeedback(left_motor.get_pos_feedback());
                Serial.print("feedback1: ");
                Serial.print(encoder1_pid.getFeedback());
                Serial.print("\t");
                Serial.print("feedback2: ");
                Serial.print(encoder2_pid.getFeedback());
                Serial.print("\n");
                int speed1 = encoder1_pid.compute();
                int speed2 = encoder2_pid.compute();
                encoder1_pid.direction > 0 ? right_motor.forward(speed1) : right_motor.backward(speed1);
                encoder2_pid.direction > 0 ? left_motor.forward(speed2) : left_motor.backward(speed2);
            }
        }

        bool isStopped(){
            if(millis() - prev_time >= stopping_time){
                if(encoder1_pid.getError() == encoder1_prev_error && encoder2_pid.getError() == encoder2_prev_error){
                    return true;
                }
                encoder1_prev_error = encoder1_pid.getError();
                encoder2_prev_error = encoder2_pid.getError();
                prev_time = millis();
            }
            return false;
        }
};

#endif
