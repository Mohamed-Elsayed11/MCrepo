#ifndef IMU_H
#define IMU_H
#include <Arduino_LSM6DSOX.h>
#include <MahonyAHRS.h>

class IMU2040
{

private:
    Mahony filter;
    float sampleFreq;
    unsigned long previous_time;
    float initial_yaw, gyro_biasX, gyro_biasY, gyro_biasZ;
    bool first_reading;
    float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
    const float gyro_threshold = 0.05;
    const float  yaw_multiplier = 1.0;
    float relative_yaw; 
    void calibrateGyroscope();

public:
    IMU2040();
    bool  init();
    void  calulations();
    float get_Yaw_angle();
};

#endif