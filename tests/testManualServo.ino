#include<Servo.h>
#include<Wire.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BNO055.h>
#include<MadgwickAHRS.h>
#include<IMU.h>

//servo
Servo SVe;
Servo SVal;
Servo SVar;
int pin_num_SVe = 1;
int pin_num_SVal = 2;
int pin_num_SVar = 3;
int pin_num_SV_Ph = 4;
int pin_num_SV_Pl = 5;
int pw_min_sg90 = 500;
int pw_max_sg90 = 2400;
int deg_min_sg90 = 0;
int deg_max_sg90 = 180;

//motor
int pin_num_MT = 6;
double Duty_max_MT =;
double Duty_min_MT =;
double thrust_max =;
double thrust_min =;
double T = 0;

//switch of autopilot
int pin_num_switch_h = 7;
int pin_num_switch_8 = 8;

//reciever
int pin_num_SVe_R = 9;
int pin_num_SVa_R = 10;
int pin_num_MT_R = 11;
int pin_num_switch_R = 12;
volatile unsigned long startTime_de=0,startTime_da=0,startTime_MT=0;
volatile unsigned long PW_de=0,PW_da=0,PW_MT=0;
volatile unsigned long Period_de=0,Period_da=0,Period_MT=0;
volatile unsigned long Duty_de=0,Duty_da=0,Duty_MT=0;

//LED
int pin_num_LED_H = 13;
int pin_num_LED_L = 14;

//nine axis sensor
  Adafruit_BNO055 bno = Adafruit_BNO055(55);
  Madgwick Madgwick;
  unsigned long previousTime=0;

void setup() {
  // put your setup code here, to run once:
//servo
  SVe.attach(pin_num_SVe,pw_min_sg90,pw_max_sg90);
  SVal.attach(pin_num_SVal,pw_min_sg90,pw_max_sg90);
  SVar.attach(pin_num_SVar,pw_min_sg90,pw_max_sg90);

//motor
  pinMode(pin_num_MT,OUTPUT);

//switch of autopilot
  pinMode(pin_num_switch_h,INPUT);
  pinMode(pin_num_switch_8,INPUT);

//reciever
  pinMode(pin_num_SVe_R,INPUT);
  pinMode(pin_num_SVa_R,INPUT);
  pinMode(pin_num_MT_R,INPUT);
  pinMode(pin_num_switch_R,INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_num_SVe_R),pulseISR_de,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_num_SVa_R),pulseISR_da,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_num_MT_R),pulseISR_MT,CHANGE);

//nine axis sensor
  Serial.begin(115200);
  Madgwick.begin(100.0f);

//LED
  pinMode(pin_num_LED_H,OUTPUT);
  pinMode(pin_num_LED_L,OUTPUT);
}

void pulseISR_de(){
  static unsigned long endTime_de=0;
  unsigned long currentTime=micros();
  if(digitalRead(pin_num_SVe_R)==HIGH){
    startTime_de = currentTime;
    Period_de = currentTime-endTime_de;
    endTime_de = currentTime;
  }else{
    PW_de = micros()-startTime_de;
  }

  if(Period_de>0){
    Duty_de=(float)PW_de/(float)Period_de;
  }else{
    Duty_de=0.0;
  }
}



void loop() {
  // put your main code here, to run repeatedly:
  int state_switch_h = digitalRead(pin_num_switch_h);
  int state_switch_8 = digitalRead(pin_num_switch_8);

  if(state_switch_h == HIGH){
  //horizontal rotation
    digitalWrite(pin_num_LED_H,HIGH);
    digitalWrite(pin_num_LED_L,LOW);
  }else if(state_switch_8 == HIGH){
  //8 rotation
    digitalWrite(pin_num_LED_H,HIGH);
    digitalWrite(pin_num_LED_L,LOW);

////////////////////////////////////////////////////////////////////

  }else{
  //proportional control
    double de = (deg_max_sg90-deg_min_sg90)*Duty_de+deg_min_sg90;

//servo
    SVe.write(de);

  }
}
