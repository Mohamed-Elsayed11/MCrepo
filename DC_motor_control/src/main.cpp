#include <Arduino.h>
#define input1 4
#define input2 5
#define enable 6
#define encoder1 2
#define encoder2 3

class DC_MOTOR{
  private:
    int in1;
    int in2;
    int EN;
    int enc1;
    int enc2;
    int speed = 255;
  public:
    DC_MOTOR(int In1, int In2, int Enable, int ENC1, int ENC2){
      in1 = In1;
      in2 = In2;
      EN = Enable;
      enc1 = ENC1;
      enc2 = ENC2;
    }
    void init(){
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(EN, OUTPUT);
      pinMode(enc1, INPUT);
      pinMode(enc2, INPUT);
    }
    void setSpeed(int Speed){
      speed = Speed;
    }
    void forward(){
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(EN, speed);
    }
    void forward(int Speed){
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(EN, Speed);
    }
    void backward(){
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(EN, speed);
    }
    void backward(int Speed){
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(EN, Speed);
    }
    void stop(){
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(EN, 0);
    }
};

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
