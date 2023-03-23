#include "Arduino.h"
#include "MotorLib.h"

void Motor::init(int _FirstEncoder,int _SecondEncoder, int _PWMOut, int _RotOut1, int _RotOut2)
{
    Timer  = millis();

    FirstEncoder = _FirstEncoder;
    SecondEncoder = _SecondEncoder;

    PWMOut = _PWMOut;

    RotOut1 = _RotOut1;
    RotOut2 = _RotOut2;

    FlagInterrapt = false;

    count = 0;
    count_prev = 0;
    delta = 0;
    Velocity = 0;
    GoalVelocity = 0;

    StateFirstEncoder = false;
    StateSecondEncoder = false;

    error = 0;
    integral = 0;
    prev_error = 0;
    D = 0;
    kp = 0.75; // 0.75 0.5 0
    ki = 1;
    kd = 0 ;
    out = 0;

    _err_measure = 5;
    _q = 0.20;

    pinMode(RotOut1,OUTPUT);
    pinMode(RotOut2,OUTPUT);
    pinMode(FirstEncoder,INPUT_PULLUP);
    pinMode(SecondEncoder,INPUT_PULLUP);
    digitalWrite(RotOut1,LOW);
    digitalWrite(RotOut2,LOW);
}
float Motor::GetRealVelocity()
{
    return Velocity;
}
void Motor::SetVolocity(float _GoalVelocity)
{
    GoalVelocity = _GoalVelocity;
}
void Motor::tick()
{
    if(FlagInterrapt)
    {
        StateFirstEncoder = digitalRead(FirstEncoder);
        StateSecondEncoder = digitalRead(SecondEncoder);
        if(StateFirstEncoder == StateSecondEncoder)
            count++;
        else
            count--;
        FlagInterrapt = false;
    }
    calculateRotSpeed();
}
