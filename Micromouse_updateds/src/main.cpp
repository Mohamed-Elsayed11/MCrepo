#include <Arduino.h>
#include "DC_MOTOR.h"
#include "PID.h"
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

DC_MOTOR motor1 = DC_MOTOR(input1_1, input2_1, enable_1, encoder1_1, encoder2_1);
DC_MOTOR motor2 = DC_MOTOR(input1_2, input2_2, enable_2, encoder1_2, encoder2_2);
PID pid1 = PID(0.55, 0.0025, 0.001); //distance control parameters for motor1
PID pid2 = PID(0.55, 0.0025, 0.001); //distance control parameters for motor2
// PID pid = PID(0.45, 0.0025, 0.001); //Rotation   control parameters
// IMU2040 imu=IMU2040();

void setup()
{
    Serial.begin(9600);
    // while (!Serial)
    //     ;
    // if (!(imu.imu_init()))
    // {
    //     Serial.println("IMU initialization failed!");
    //     while (1)
    //         ;
    // }
     motor1.init();
     motor2.init();
     pid1.setSetpoint(1000);
     pid2.setSetpoint(1000);
    //  pid.setSetpoint(90);
}

void loop()
{
    int feedback1 = motor1.get_pos_feedback();
    int feedback2 = motor2.get_pos_feedback();

    Serial.print("feedback1: ");
    Serial.print(feedback1);
    Serial.print("\t");
    Serial.print("feedback2: ");
    Serial.print(feedback2);
    Serial.print("\n");

    int speed1 = pid1.compute(feedback1);
    int speed2 = pid2.compute(feedback2);

    pid1.direction > 0 ? motor1.forward(speed1) : motor1.backward(speed1);
    pid2.direction > 0 ? motor2.forward(speed2) : motor2.backward(speed2);
 
    // imu.imu_calulations();
    // float yaw = imu.get_Yaw_angle();
    // Serial.print("Current Relative Yaw: ");
    // Serial.print(yaw);
    // Serial.println(" degrees");
    // int speed =pid.compute(yaw);
    // pid.direction > 0 ? motor1.forward(speed) : motor1.backward(speed);
    // delay(500);
}