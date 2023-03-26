#include "Arduino.h"
#include "MotorLib.h"
#include "Config.h"
void Motor::init(int _FirstEncoder, int _SecondEncoder, int _PWMOut, int _RotOut1, int _RotOut2)
{
  Timer = millis();

  FirstEncoder = _FirstEncoder;
  SecondEncoder = _SecondEncoder;

  RecognitionTime = RecognitionTimeofEncoder;

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
  kp = KP; // 0.75 0.5 0
  ki = KI;
  kd = KD;
  out = 0;

  _err_measure = 1;
  _q = 0.020;

  pinMode(RotOut1, OUTPUT);
  pinMode(RotOut2, OUTPUT);
  pinMode(FirstEncoder, INPUT_PULLUP);
  pinMode(SecondEncoder, INPUT_PULLUP);
  digitalWrite(RotOut1, LOW);
  digitalWrite(RotOut2, LOW);
  // attachInterrupt(digitalPinToInterrupt(FirstEncoder),FlagInterrupt,CHANGE);
}
float Motor::GetRealVelocity()
{
  return Velocity;
}
void Motor::SetVolocity(float _GoalVelocity)
{
  this->GoalVelocity = _GoalVelocity;
}
void Motor::Flag()
{
  this->FlagInterrapt = true;
}
void Motor::tick()
{
  if (this->FlagInterrapt)
  {
    this->StateFirstEncoder = digitalRead(FirstEncoder);
    this->StateSecondEncoder = digitalRead(SecondEncoder);
    //Serial.print(StateFirstEncoder);
    // Serial.println(StateSecondEncoder);
    if (StateFirstEncoder == StateSecondEncoder)
    {
      count++;
    }
    else
    {
      count--;
    }
    this->FlagInterrapt = false;
  }
  calculateRotSpeed();
}
void Motor::calculateRotSpeed()
{
  if (millis() - Timer >= RecognitionTime)
  {
    Timer  = millis();
    delta = count - count_prev;
    count_prev = count;
    float Velocity_temp = delta * 1000 / RecognitionTime * RATIO;
    Velocity = simpleKalman(Velocity_temp);
    VelocityPID(GoalVelocity, Velocity);
  }
}
float Motor::simpleKalman(float newVal)
{
  float _kalman_gain, _current_estimate;
  static float _err_estimate = _err_measure;
  static float _last_estimate;
  _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
  _err_estimate = (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;
  return _current_estimate;
}

void Motor::VelocityPID(float GoalVelocity, float Velocity)
{
  error = GoalVelocity - Velocity;
  integral = integral + error * RecognitionTime / 1000 * ki;
  D = error * 1000 / RecognitionTime;
  out = error * kp + integral + D * kd;
  Send2Driver3Pin(out);
}
void Motor::Send2Driver2Pin(int Signal)
{
  if (Signal > 255)
    NewV = 255;
  if (Signal < -255)
    NewV = -255;
  if (Signal < 0 )
  {
    analogWrite(PWMOut, 255 + Signal);
    digitalWrite(RotOut2, HIGH);
  }
  else
  {
    analogWrite(PWMOut, Signal);
    digitalWrite(RotOut2, LOW);
  }
}
void Motor::Send2Driver3Pin(float V)
{
  int NewV = abs(V);
  if (NewV > 255)
    NewV = 255;

  if (V > 0 )
  {
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);

  }
  else if (V < 0)
  {
    digitalWrite(5, HIGH);
    digitalWrite(6, LOW);
  }
  analogWrite(PWMOut, NewV);
}
