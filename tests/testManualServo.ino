#include<Servo.h>
#include<Wire.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BNO055.h>
#include<MadgwickAHRS.h>

//servo
Servo SVal;
int pin_num_SVal = 9;
int deg_min_sg90 = 0;
int deg_max_sg90 = 180;

//reciever

int pin_num_SVa_R = 2;

volatile uint32_t startTime_de=0,startTime_da=0,startTime_MT=0;
volatile uint32_t PW_de=0,PW_da=0,PW_MT=0;
volatile uint32_t Period_de=0,Period_da=0,Period_MT=0;
volatile float Duty_de=0.0,Duty_da=0.0,Duty_MT=0.0;

void setup() {
  // put your setup code here, to run once:
//servo
  //SVal.attach(pin_num_SVal);
    pinMode(pin_num_SVal,OUTPUT);

//reciever
  pinMode(pin_num_SVa_R,INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_num_SVa_R),pulseISR_da,CHANGE);


 Serial.begin(115200);

}

void pulseISR_da(){
  static uint32_t endTime_da=0;
  uint32_t currentTime=micros();
  if(digitalRead(pin_num_SVa_R)==HIGH){
    startTime_da = currentTime;
    Period_da = currentTime-endTime_da;
    endTime_da = currentTime;
  }else{
    PW_da = currentTime-startTime_da;
  }

  if(Period_da>0){
    Duty_da=(double)PW_da/(double)Period_da;
    Duty_da = constrain(Duty_da, 0, 1);
  }else{
    Duty_da=0.0;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
/*
  noInterrupts();
  uint32_t pulseWidth = PW_da;
  uint32_t period = Period_da;
  interrupts();

    Duty_da = (float)pulseWidth/(float)period;
    Duty_da = constrain(Duty_da, 0, 1);
*/
  //proportional control
  double da = (deg_max_sg90-deg_min_sg90)*Duty_da+deg_min_sg90;
//servo
    //SVal.write(da);
analogWrite(pin_num_SVal,255*Duty_da);

Serial.print(da,3);
  Serial.print("\t");
//Serial.print(PW_da,3);
  //Serial.print("\t");
//Serial.print(Period_da,3);
  //Serial.print("\t");
Serial.println(Duty_da,3);
delay(100);
}
