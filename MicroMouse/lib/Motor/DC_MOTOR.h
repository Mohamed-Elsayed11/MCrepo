#ifndef DC_MOTOR_H
#define DC_MOTOR_H
#define F_SIZE 10
#include <Arduino.h>

class DC_MOTOR {
public:
    DC_MOTOR(int In1, int In2, int Enable, int EncA, int EncB);
    void init();
    void setSpeed(int Speed);
    void forward();
    void forward(int Speed);
    void backward();
    void backward(int Speed);
    void stop();
    int get_pos_feedback_1();
    int get_pos_feedback_2();
    void reset_pos_1();
    void reset_pos_2();
    void update_velocity_1(unsigned long current_time);
    void update_velocity_2(unsigned long current_time);
    int get_velocity_1();
    int get_velocity_2();
    float velocity_mean_filter_1[F_SIZE]={0};
    char F_index_1 =0;
    float Filtered_velocity_1=0.0;
    char Limit_1=0;
    char Limit_2=0;
    float velocity_mean_filter_2[F_SIZE]={0};
    char F_index_2 =0;
    float Filtered_velocity_2=0.0;


private:
    static int motor_num;
    static DC_MOTOR* motor1_ptr;
    static DC_MOTOR* motor2_ptr;
    int in1, in2, EN, encA, encB;
    int speed;
    volatile long pos_1, last_pos_1, last_time_1, velocity_1;
    volatile long pos_2, last_pos_2, last_time_2, velocity_2;

    static void encoder_callback1();
    static void encoder_callback2();
    void handle_encoder_1();
    void handle_encoder_2();
};

#endif
