#include "MotorLib.h"
Motor A;
#define PWM 9
#define in1 5
#define in2 6
#define Encoder1 2
#define Encoder2 3
void Flag()
{
    A.FlagInterrapt();
}
void setup()
{
    Serial.begin(9600);
    A.init(Encoder1,Encoder2,PWM, in1,in2);
    attachInterrupt(digitalPinToInterrupt(Encoder1),Flag,CHANGE);
}
void loop()
{

}