#include <Arduino.h>
#include "PID.h"

// int pid(int error);

PID pid = PID(0.45, 0.0025, 0.001);

void setup()
{
  Serial.begin(9600);

  // attachInterrupt(digitalPinToInterrupt(ENCA), Encoder_read, RISING);
}

void loop()
{
  /*error_estimation();
  motor_control(output, dir);
  Serial.println(set_point);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
  delay(300);*/
}
/*void Encoder_read(void)
{
  int Encoder_B = digitalRead(ENCB);
  if (Encoder_B > 0)
  {
    pos++;
  }
  else
  {
    pos--;
  }
}*/
/*void motor_control(int speed, char dir)
{
  if (dir == 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if (dir == 1)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(EN, speed);
}*/

/*void error_estimation(void)
{
  now = millis();
  dt = ((float)(now - last_time) / 1000);
  last_time = now;
  error = set_point - pos;
  controlled_signal = pid(error);
  output = fabs(controlled_signal);
  if (output > 255)
  {
    output = 255;
  }
  if (controlled_signal < 0)
  {
    dir = 1;
  }
  else
  {
    dir = 0;
  }
}
int pid(int error)
{
  integral += error * dt;
  derivative = (error - pervious_error) / dt;
  pervious_error = error;
  return (kp * error + ki * integral + kd * derivative);
}*/
