#include <Arduino.h>
#include <DC_MOTOR.h>

#define input1 4
#define input2 5
#define enable 6
#define encoder1 2
#define encoder2 3

DC_MOTOR motor1 = DC_MOTOR(input1, input2, enable,encoder1, encoder2);

void setup() {
  motor1.init();
}

void loop() {
  motor1.setSpeed(100);
  motor1.forward();
  delay(1000);
  motor1.stop();
  delay(1000);
}
