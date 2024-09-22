#include <Arduino.h>
#include <Adafruit_VL6180X.h>
#include "IMU.h"

MbedI2C I2C1(A0, A1);

#define TOF1_ADDRESS 0x30
#define TOF2_ADDRESS 0x31

#define SHUT1 A2
#define SHUT2 A3

Adafruit_VL6180X right_TOF = Adafruit_VL6180X();
Adafruit_VL6180X left_TOF = Adafruit_VL6180X();

IMU2040 imu = IMU2040();

void setID() {
  digitalWrite(SHUT1, HIGH);
  digitalWrite(SHUT2, LOW);

  right_TOF.begin(&I2C1);
  right_TOF.setAddress(TOF1_ADDRESS);
  delay(10);
  digitalWrite(SHUT2, HIGH);
  delay(10);

  left_TOF.begin(&I2C1);
  left_TOF.setAddress(TOF2_ADDRESS);
}

void setup() {
  Serial.begin(115200);
  pinMode(SHUT1, OUTPUT);
  pinMode(SHUT2, OUTPUT);
  setID();
  imu.init();
}

void loop() {
  for(int i = 0; i < 1000; i++){
    Serial.print("sensor 1: ");
    Serial.print(right_TOF.readRange());
    Serial.print('\t');
    Serial.print("sensor 2: ");
    Serial.print(left_TOF.readRange());
    Serial.print("\n");
  }

  for(int i = 0; i < 1000; i++){
    imu.calulations();
    Serial.print("yaw: ");
    Serial.println(imu.get_Yaw_angle());
    Serial.println("============================================");
  }
}