#include<Wire.h>

//reciever
int pin_num_SVa_R = 2;
int pin_num_MT_R = 3;
volatile float startTime_de=0,startTime_da=0,startTime_MT=0;
volatile float PW_de=0,PW_da=0,PW_MT=0;
volatile float Period_de=0,Period_da=0,Period_MT=0;
volatile float Duty_de=0,Duty_da=0,Duty_MT=0;

void setup() {
  // put your setup code here, to run once:
//servo

//reciever
  pinMode(pin_num_SVa_R,INPUT);
  pinMode(pin_num_MT_R,INPUT);


  attachInterrupt(digitalPinToInterrupt(pin_num_SVa_R),pulseISR_da,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_num_MT_R),pulseISR_MT,CHANGE);

Serial.begin(9600);
}



void pulseISR_da(){
  static unsigned long endTime_da=0;
  unsigned long currentTime=micros();
  if(digitalRead(pin_num_SVa_R)==HIGH){
    startTime_da = currentTime;
    Period_da = currentTime-endTime_da;
    endTime_da = currentTime;
  }else{
    PW_da = micros()-startTime_da;
  }

  if(Period_da>0){
    Duty_da=(float)PW_da/(float)Period_da;
  }else{
    Duty_da=0.0;
  }
}

void pulseISR_MT(){
  static unsigned long endTime_MT=0;
  unsigned long currentTime=micros();
  if(digitalRead(pin_num_MT_R)==HIGH){
    startTime_MT = currentTime;
    Period_MT = currentTime-endTime_MT;
    endTime_MT = currentTime;
  }else{
    PW_MT = micros()-startTime_MT;
  }

  if(Period_MT>0){
    Duty_MT=(float)PW_MT/(float)Period_MT;
  }else{
    Duty_MT=0.0;
  }
}

void loop() {
Serial.print(Duty_da,3);
  Serial.print("\t");
Serial.println(Duty_MT,3);

delay(250);
}
