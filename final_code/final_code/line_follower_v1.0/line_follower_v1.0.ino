int PWMA = 17 , PWMB = 18 , VCC = 16;
int sensor_power = 23;
int Sensors[4]={39,34,35,32};
int Readings[4]={};
int Strength[4]={};
int white[4]={0,0,0,0};
int black[4]={4095,4095,4095,4095};
int weight[4]={0,1000,2000,3000};
int center = 1500;
int base_speed = 180;
int min_speed = 30;
int left_speed = 180,right_speed=180;
int error,prev_error = 0;
float proportional = 0;
float kp = 0.08;
float integral = 0;
float ki = 0.0008;
float derivative = 0;
float kd = 0.08;
float correction = 0.0;
unsigned long previous_time = 0;
const unsigned long sample_time = 10;
unsigned long current_time;
bool line_lost = false;

void collect_readings()
{
  for(int i=0;i<4;i++)
  {
    Readings[i] = analogRead(Sensors[i]);
  }
}

int calculate_position(int Strength[])
{
  int numerator = (Strength[0]*weight[0])+(Strength[1]*weight[1])+(Strength[2]*weight[2])+(Strength[3]*weight[3]);
  int denominator = (Strength[0]+Strength[1]+Strength[2]+Strength[3]);
  int position;
  if(denominator != 0) position = numerator/denominator;
  else position = -1;
  return position;
}

void calculate_strength(int Readings[])
{
  for(int i=0;i<4;i++)
  {
    if(white[i]!=black[i])
    {
      Strength[i]= (white[i]-Readings[i])*1000/(white[i]-black[i]);
      Strength[i] = constrain(Strength[i],0,1000);
    }
    else Strength[i]=0;
  }
}

void calibrate()
{
  unsigned long startTime = millis();
  while(millis()-startTime <= 10000)
  {
    collect_readings();
    for(int j=0;j<4;j++)
    {
      white[j]=max(white[j],Readings[j]);
      black[j]=min(black[j],Readings[j]);
    }
    delay(5);
  }
}

void calculate_error(int position)
{
  error = position - center;
}

void PID()
{
  float dt = (current_time-previous_time)/1000.0;

  proportional  = kp*error;
  integral += ki*error*dt;
  integral = constrain(integral,-50,50);
  derivative = kd*(error-prev_error)/dt;
  prev_error = error;
  correction = proportional + integral + derivative;
  correction = constrain(correction,-127.5,127.5);
  

}

void speed_adjustment()
{
  base_speed = min(180,255-(int)abs(correction));
  left_speed = base_speed + correction;
  right_speed = base_speed - correction;
  left_speed = constrain(left_speed,min_speed,255);
  right_speed = constrain(right_speed,min_speed,255);
}

void motor_control()
{
  analogWrite(PWMA,right_speed);
  analogWrite(PWMB,left_speed);
}

void setup() {
  // put your setup code here, to run once:
  // pin and sensor config
  // pinMode(AI1,OUTPUT);
  // pinMode(AI2,OUTPUT);
  // pinMode(BI1,OUTPUT);
  // pinMode(BI2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(VCC,OUTPUT);
  pinMode(sensor_power,OUTPUT);
  for(int i=0;i<4;i++)
  {
    pinMode(Sensors[i],INPUT);
  }

  // digitalWrite(AI1,HIGH);
  // digitalWrite(AI2,LOW);
  // digitalWrite(BI1,HIGH);
  // digitalWrite(BI2,LOW);
  digitalWrite(VCC,LOW);
  // analogWrite(PWMA,255);
  // analogWrite(PWMB,255);
  digitalWrite(sensor_power,HIGH);

  calibrate();
  digitalWrite(VCC,HIGH);

  previous_time = millis();

}

void loop() {
  // put your main code here, to run repeatedly:
  current_time = millis();
  if(current_time - previous_time >= sample_time)
  {
    collect_readings();
    calculate_strength(Readings);

    int position = calculate_position(Strength);

    if(position == -1)
    {
      // line loss
      left_speed = 0;
      right_speed = 0;

      integral = 0;
      prev_error = 0;
      line_lost = true;
      derivative = 0;

      motor_control();
    }
    else
    {
      calculate_error(position);

      if(line_lost)
      {
        prev_error = error;
        line_lost = false;
      }

      PID();
      speed_adjustment();

      motor_control();
    }

    

    previous_time = current_time;

  }

}
