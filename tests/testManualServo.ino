#include<Servo.h>
#include<Wire.h>

//servo
Servo SVal;

int pin_num_SVal = 9;

//int pw_min_sg90 = 500;
//int pw_max_sg90 = 2400;
int deg_min_sg90 = 0;
int deg_max_sg90 = 180;

//reciever

int pin_num_SVa_R = 2;

volatile float startTime_de=0,startTime_da=0,startTime_MT=0;
volatile float PW_de=0,PW_da=0,PW_MT=0;
volatile float Period_de=0,Period_da=0,Period_MT=0;
volatile float Duty_de=0,Duty_da=0,Duty_MT=0;

void setup() {
  // put your setup code here, to run once:
//servo
  SVal.attach(pin_num_SVal);

//reciever
  pinMode(pin_num_SVa_R,INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_num_SVa_R),pulseISR_da,CHANGE);


 Serial.begin(115200);

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


void loop() {
  // put your main code here, to run repeatedly:
  //proportional control
    double da = 2*((deg_max_sg90-deg_min_sg90)*Duty_da+deg_min_sg90);
da = constrain(da, 0, 180);
//servo
    SVal.write(da);

Serial.print(da,3);
  Serial.print("\t");
Serial.print(PW_da,3);
  Serial.print("\t");
Serial.print(Period_da,3);
  Serial.print("\t");
Serial.println(Duty_da,3);
delay(200);
}
