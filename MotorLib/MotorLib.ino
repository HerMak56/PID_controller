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
  A.init(Encoder1, Encoder2, PWM, in1, in2);
  attachInterrupt(digitalPinToInterrupt(Encoder1), FlagInter, CHANGE);
  //Serial.println("Goal, Real");
}
void FlagInter()
{
  //Serial.println(1);
  A.Flag();
}
void loop()
{
  A.tick();
  float GoalVel = map(analogRead(A1), 0, 1024, -90, 90);
  A.SetVolocity(GoalVel);
}
