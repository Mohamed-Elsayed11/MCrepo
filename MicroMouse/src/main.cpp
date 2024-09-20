#include <Arduino.h>

#include <Wire.h>
#include <VL6180X.h>

MbedI2C I2C1(A0, A1);
VL6180X sensor;
VL6180X sensor2;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  I2C1.begin();

  sensor2.setBus(&I2C1);
  sensor.init();
  sensor2.init();
  sensor.configureDefault();
  sensor2.configureDefault();

  // Reduce range max convergence time and ALS integration
  // time to 30 ms and 50 ms, respectively, to allow 10 Hz
  // operation (as suggested by table "Interleaved mode
  // limits (10 Hz operation)" in the datasheet).
  sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor2.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor2.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);

  sensor.setTimeout(100);
  sensor2.setTimeout(100);

   // stop continuous mode if already active
  sensor.stopContinuous();
  sensor2.stopContinuous();
  // in case stopContinuous() triggered a single-shot
  // measurement, wait for it to complete
  delay(300);
  // start interleaved continuous mode with period of 100 ms
  sensor.startInterleavedContinuous(100);
  sensor2.startInterleavedContinuous(100);
}

void loop()
{
  Serial.print("\tRange: ");
  Serial.print(sensor.readRangeContinuousMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.print("\t\t");

  Serial.print("\tRange: ");
  Serial.print(sensor2.readRangeContinuousMillimeters());
  if (sensor2.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
}