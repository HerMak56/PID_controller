#include "MotorLib.h"
Motor A;
#define PWM 9
#define in1 5
#define in2 6
#define Encoder1 2
#define Encoder2 3
void setup()
{
    Serial.begin(9600);
    A.init(Encoder1,Encoder2,PWM, in1,in2);
    attachInterrupt(digitalPinToInterrupt(Encoder1),FlagInter,CHANGE);
    pinMode(A1,INPUT);
}
void FlagInter()
{
    A.Flag();
}
void loop()
{
    float GoalVel = map(analogRead(A1),0,1024,-75,75);
    A.tick();
    A.SetVolocity(GoalVel);
    Serial.print(GoalVel);
    Serial.print(',');
    Serial.println(A.GetRealVelocity());
}