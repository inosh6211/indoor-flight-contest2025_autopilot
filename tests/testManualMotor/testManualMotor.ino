#include<Servo.h>
#include<Wire.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BNO055.h>
#include<MadgwickAHRS.h>

//motor
int pin_num_MT = 9;
double Duty_max_MT =1;
double Duty_min_MT =0;
double thrust_max =0;
double thrust_min =0;
double T = 0;

//reciever
int pin_num_MT_R = 2;
volatile uint32_t startTime_de=0,startTime_da=0,startTime_MT=0;
volatile uint32_t PW_de=0,PW_da=0,PW_MT=0;
volatile uint32_t Period_de=0,Period_da=0,Period_MT=0;
volatile float Duty_de=0.0,Duty_da=0.0,Duty_MT=0.0;

void setup() {
  // put your setup code here, to run once:

//motor
  pinMode(pin_num_MT,OUTPUT);

//reciever
  pinMode(pin_num_MT_R,INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_num_MT_R),pulseISR_MT,CHANGE);


  Serial.begin(115200);

}

void pulseISR_MT(){
  static uint32_t endTime_MT=0;
  uint32_t currentTime=micros();
  if(digitalRead(pin_num_MT_R)==HIGH){
    startTime_MT = currentTime;
    Period_MT = currentTime-endTime_MT;
    endTime_MT = currentTime;
  }else{
    PW_MT = currentTime-startTime_MT;
  }

  if(Period_MT>0){
    Duty_MT=(double)PW_MT/(double)Period_MT;
    Duty_MT = constrain(Duty_MT, 0, 1);
  }else{
    Duty_MT=0.0;
  }
}



void loop() {
  // put your main code here, to run repeatedly:

//motor
    analogWrite(pin_num_MT,Duty_MT*255);

  Serial.print(1);
  Serial.print("\t");
//Serial.print(PW_da,3);
  //Serial.print("\t");
//Serial.print(Period_da,3);
  //Serial.print("\t");
Serial.println(Duty_MT,3);
delay(100);
}
