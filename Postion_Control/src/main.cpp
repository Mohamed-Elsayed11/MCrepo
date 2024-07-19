#include <Arduino.h>
#define ENCA 2
#define ENCB 3
#define EN 6
#define IN1 4
#define IN2 5
int pos = 0;
float kp = 0.45, ki = 0.0025, kd = 0.001;
int set_point = 1500;
char dir = 0;
float integral = 0, derivative = 0, error = 0, pervious_error = 0, controlled_signal = 0, output = 0;
long long now = 0, last_time = 0;
float dt = 0;
void Encoder_read(void);
void motor_control(int speed, char dir);
void error_estimation(void);
int pid(int error);
void setup()
{
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(EN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), Encoder_read, RISING);
}

void loop()
{
  error_estimation();
  motor_control(output, dir);
  Serial.println(set_point);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
  delay(300);
}
void Encoder_read(void)
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
}
void motor_control(int speed, char dir)
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
}
void error_estimation(void)
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
}
