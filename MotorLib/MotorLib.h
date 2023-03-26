#pragma once

class Motor
{
public:
    // Val for pins
    int FirstEncoder;
    int SecondEncoder;
    int PWMOut;
    int RotOut1;
    int RotOut2;

    // Val for flags
    bool FlagInterrapt;

    // Val for calculate vel
    int count;
    int count_prev; // for calculate delta
    float delta;
    float Velocity;
    float GoalVelocity;

    // Timer
    uint32_t Timer;

    // Val for state Encoders
    bool StateFirstEncoder;
    bool StateSecondEncoder;

    float RecognitionTime;

    // Val for PID
    double error;
    double integral;
    double prev_error;
    double D;
    double kp; // 0.75 0.5 0
    double ki;
    double kd;
    double out;

    // Val for filter
    float _err_measure;
    float _q;

    void Send2Driver2Pin(int Signal);
    void calculateRotSpeed();
    void VelocityPID(float GoalVelocity, float Velocity);
    float simpleKalman(float newVal);

    int NewV;
    void init(int FirstEncoder, int SecondEncoder, int PWMOut, int RotOut1, int RotOut2);
    void Flag();
    void tick();
    void SetVolocity(float GoalVelocity);
    float GetRealVelocity();
    void Send2Driver3Pin(float Signal);
};
