#define PWM 9
#define in1 5
#define in2 6
#define Inter1 2
#define Inter2 3
#define RecognitionTime 20 // time in milliseconds;

bool Flag = false;// flag interrapt

int count = 0;// count of tikcks
int count_prev = 0; // for calculate delta
float delta = 0;
float Velocity = 0;
uint32_t Time = millis();

bool First = false;
bool Second  = false;
bool stat1 = false;
bool stat2 = false;

double error = 0;
double integral = 0;
double prev_error = 0;
double D = 0;
double kp = 0.75; // 0.75 0.5 0
double ki = 1;
double kd = 0 ;
double out = 0;

float GoalVelocity = 0;

float _err_measure = 5;  // примерный шум измерений
float _q = 0.02;   // скорость изменения значений 0.001-1, варьировать самому

void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(Inter1), FlagInterrupt, CHANGE);
  Serial.setTimeout(5);
  digitalWrite(5, LOW);
  digitalWrite(6, HIGH);
  analogWrite(9, 0);
  Serial.println("RealVelocity, Goalvelocity");
}
void FlagInterrupt()
{
  Flag = true;
}
void Send2Driver(float V)
{
  int NewV = int(255 * abs(V) / 12);
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
  else
  {
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
  }
  analogWrite(PWM, NewV);
}
void StatsFlag()
{
  if (Flag)
  {

    First = digitalRead(2);
    Second = digitalRead(3);
    //Serial.print(First);
    //Serial.println(Second);
    if (First == Second)
    {
      count++;
      //Serial.println(count);
    }
    else
    {
      count--;
     // Serial.println(count);
    }
    Flag = false;
    //Serial.println(count);
  }
}
void calculateRotSpeed()
{
  if (millis() - Time >= RecognitionTime)
  {
    Time = millis();
    delta = count - count_prev;
    count_prev = count;
    float Velocity_temp = delta * (1000 / RecognitionTime) * 0.0285;
    Velocity = simpleKalman(Velocity_temp);
    GoalVelocity = map(analogRead(A1), 0, 1024, -75, 75);
    VelocityPID(GoalVelocity, Velocity);
    Serial.println(Velocity);
    Serial.print(',');
    Serial.println(GoalVelocity);
  }
}
void VelocityPID(double GoalVelocity, double Velocity)
{
  error = GoalVelocity - Velocity;
  integral = integral + error * ki * (double)RecognitionTime / 1000;
  //Serial.println(integral);
  D = (error - prev_error) / RecognitionTime * 1000;
  prev_error = error;
  out  = error * kp + integral + D * kd;
  Send2Driver(out);
  //Serial.println(out);
}
float simpleKalman(float newVal) {
  float _kalman_gain, _current_estimate;
  static float _err_estimate = _err_measure;
  static float _last_estimate;
  _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;
  return _current_estimate;
}
void loop() {
  StatsFlag();
  calculateRotSpeed();
}
