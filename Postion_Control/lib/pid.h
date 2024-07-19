

class PID
{

private:
    float kp;
    float ki;
    float kd;
    int set_point;
    float integral;
    float derivative;
    float error, pervious_error;
    float controlled_signal;
    float output;
    long long now, last_time;
    float dt;

public:
    PID(float kp, float ki, float kd);
    void error_estimation();
    int pid(int error);
};