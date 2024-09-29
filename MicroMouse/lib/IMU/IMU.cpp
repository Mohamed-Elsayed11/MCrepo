#include "IMU.h"
IMU2040::IMU2040()
{
    sampleFreq = 512.0f;
    previous_time = 0;
    initial_yaw = 0.0;
    gyro_biasX = 0.0;
    gyro_biasY = 0.0;
    gyro_biasZ = 0.0;
    first_reading = true;
}

bool IMU2040::init()
{
    if (!IMU.begin())
    {
        Serial.println("Failed to initialize IMU!");
        return false;
    }
    else
    {
        Serial.println("IMU initialized!");
        calibrateGyroscope();
        return true;
    }
}

void IMU2040::calibrateGyroscope()
{
    const int num_samples = 300;
    float sumX = 0.0, sumY = 0.0, sumZ = 0.0;

    for (int i = 0; i < num_samples; i++)
    {
        float gx, gy, gz;
        if (IMU.gyroscopeAvailable())
        {
            IMU.readGyroscope(gx, gy, gz);
            sumX += gx;
            sumY += gy;
            sumZ += gz;
        }
        delay(10);
    }

    gyro_biasX = sumX / num_samples;
    gyro_biasY = sumY / num_samples;
    gyro_biasZ = sumZ / num_samples;
}

void IMU2040::calulations()
{
    if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable())
    {
        IMU.readGyroscope(gyroX, gyroY, gyroZ);
        IMU.readAcceleration(accelX, accelY, accelZ);
        gyroX -= gyro_biasX;
        gyroY -= gyro_biasY;
        gyroZ -= gyro_biasZ;
        filtered_gyroX = LPF * gyroX + (1 - LPF) * filtered_gyroX;
        filtered_gyroY = LPF * gyroY + (1 - LPF) * filtered_gyroY;
        filtered_gyroZ = LPF * gyroZ + (1 - LPF) * filtered_gyroZ;
        filter.updateIMU(filtered_gyroX, filtered_gyroY, filtered_gyroZ, accelX, accelY, accelZ);
        float yaw = filter.getYaw();
        yaw = (yaw * yaw_multiplier);

        if (first_reading)
        {
            initial_yaw = yaw;
            first_reading = false;
        }

        relative_yaw = ((yaw - initial_yaw) * 10);
        delay(10);
    }
}
float IMU2040::get_Yaw_angle()
{
    return relative_yaw;
}
void IMU2040::reset()
{
    initial_yaw = 0.0;
    relative_yaw = 0.0;
    
    
    first_reading = true;  
    gyro_biasX = 0.0;
    gyro_biasY = 0.0;
    gyro_biasZ = 0.0;
    calibrateGyroscope();  
}
