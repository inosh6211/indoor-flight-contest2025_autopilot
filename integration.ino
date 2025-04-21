#include<Wire.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BNO055.h>
#include<MadgwickAHRS.h>
#include<Arduino.h>
#include<esp32-hal-ledc.h>

//servo
int pin_num_SVe = 27;
int pin_num_SVal = 13;
int pin_num_SVar = 14;
int channel_SVe = 0;
int channel_SVal = 1;
int channel_SVar = 2;

//motor
int pin_num_MT = 15;
double Duty_max_MT = 1;
double Duty_min_MT = 0;
double thrust_max = 20;
double thrust_min = 0;
double T = 0;
int channel_MT = 3;

//switch of autopilot
int pin_num_switch_h = 26;
volatile uint32_t startTime_S=0;
volatile uint32_t PW_S=0;
volatile uint32_t Period_S=0; 
volatile float Duty_S=0.0;

//reciever
int pin_num_SVe_R = 33;
int pin_num_SVa_R = 32;
int pin_num_MT_R = 25;
volatile uint32_t startTime_de=0,startTime_da=0,startTime_MT=0;
volatile uint32_t PW_de=0,PW_da=0,PW_MT=0;
volatile uint32_t Period_de=0,Period_da=0,Period_MT=0; 
volatile float Duty_de=0.0,Duty_da=0.0,Duty_MT=0.0;

//LED
int pin_num_LED_H = 2;

/*

//nine axis sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x29);
Madgwick Madgwick;
unsigned long previousTime = 0;

*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //servo
  pinMode(pin_num_SVal,OUTPUT);
  pinMode(pin_num_SVar,OUTPUT);
  pinMode(pin_num_SVe,OUTPUT);
  ledcAttachChannel(pin_num_SVe, 5000, 8,channel_SVe);
  ledcAttachChannel(pin_num_SVal, 5000, 8,channel_SVal);
  ledcAttachChannel(pin_num_SVar, 5000, 8,channel_SVar);

  //motor
  pinMode(pin_num_MT,OUTPUT);
  ledcAttachChannel(pin_num_MT, 5000, 8,channel_MT);

  //switch of autopilot
  pinMode(pin_num_switch_h,INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_num_switch_h),pulseISR_S,CHANGE);

  //reciever
  pinMode(pin_num_SVe_R,INPUT);
  pinMode(pin_num_SVa_R,INPUT);
  pinMode(pin_num_MT_R,INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_num_SVe_R),pulseISR_de,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_num_SVa_R),pulseISR_da,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_num_MT_R),pulseISR_MT,CHANGE);

/*

  //nine axis sensor
  Wire.begin(21, 22);  // SDA=21, SCL=22 に明示指定
  bno.begin();
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring!");
    while (1);
  }
  Madgwick.begin(100.0f);

*/

  //LED
  pinMode(pin_num_LED_H,OUTPUT);
}

void pulseISR_S(){
  static uint32_t endTime_S=0;
  uint32_t currentTime=micros();
  if(digitalRead(pin_num_switch_h)==HIGH){
    startTime_S = currentTime;
    Period_S = currentTime-endTime_S;
    endTime_S = currentTime;
  }else{
    PW_S = currentTime-startTime_S;
  }
}

void pulseISR_de(){
  static uint32_t endTime_de=0;
  uint32_t currentTime=micros();
  if(digitalRead(pin_num_SVe_R)==HIGH){
    startTime_de = currentTime;
    Period_de = currentTime-endTime_de;
    endTime_de = currentTime;
  }else{
    PW_de = currentTime-startTime_de;
  }
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
}

float de(float AoA, float Q){
  //control law of elevator
  float Kp = 1;
  float Kd = 1;
  float AoA_tar = 0;
  float Cm_tar = 0.00961;
  float Cm_de_tar = 0.327;
  float de_0 = Cm_tar/Cm_de_tar;

  float de = Kp*(AoA_tar-AoA)-Kd*Q+de_0;
  return de;
}

float da(float phi, float P){
  //control law of aileron
  float Kp = 1;
  float Kd = 1;
  float phi_tar = 8;

  float da = Kp*(phi_tar-phi)-Kd*P;
  return da;
}

float thrust(float T, float Vc, float AoA, float B, float theta, float psi, float phi, float Twb[3][3]){
  //control law of motor
  float Kp = 1;
  float Kd = 1;
  float M = 0.25;
  float g = 9.81;
  float S = 0.1419;
  float rho = 1.2;

  float AeroCoeffs[12][3] = {
    //AoA,CL,CD
    {-6,0.09570869,0.07847764},
    {-4,0.2594904,0.07496501},
    {-2,0.4268429,0.07774973},
    {0,0.5888926,0.08518272},
    {2,0.696649,0.100125},
    {4,0.8238163,0.1190062},
    {6,0.9400842,0.1417616},
    {8,1.043605,0.167131},
    {10,1.111612,0.1952314},
    {12,1.121402,0.2307276},
  };

  float CL = 0;
  float CD = 0;

  if(-6<AoA && AoA<=-4){
    CL = (AeroCoeffs[1][1]-AeroCoeffs[0][1])/(AeroCoeffs[1][0]-AeroCoeffs[0][0])*(AoA-AeroCoeffs[0][0])+AeroCoeffs[0][1];
    CD = (AeroCoeffs[1][2]-AeroCoeffs[0][2])/(AeroCoeffs[1][0]-AeroCoeffs[0][0])*(AoA-AeroCoeffs[0][0])+AeroCoeffs[0][2];
  }else if(-4<AoA && AoA<=-2){
    CL = (AeroCoeffs[2][1]-AeroCoeffs[1][1])/(AeroCoeffs[2][0]-AeroCoeffs[1][0])*(AoA-AeroCoeffs[1][0])+AeroCoeffs[1][1];
    CD = (AeroCoeffs[2][2]-AeroCoeffs[1][2])/(AeroCoeffs[2][0]-AeroCoeffs[1][0])*(AoA-AeroCoeffs[1][0])+AeroCoeffs[1][2];
  }else if(-2<AoA && AoA<=0){
    CL = (AeroCoeffs[3][1]-AeroCoeffs[2][1])/(AeroCoeffs[3][0]-AeroCoeffs[2][0])*(AoA-AeroCoeffs[2][0])+AeroCoeffs[2][1];
    CD = (AeroCoeffs[3][2]-AeroCoeffs[2][2])/(AeroCoeffs[3][0]-AeroCoeffs[2][0])*(AoA-AeroCoeffs[2][0])+AeroCoeffs[2][2];
  }else if(0<AoA && AoA<=2){
    CL = (AeroCoeffs[3][1]-AeroCoeffs[2][1])/(AeroCoeffs[3][0]-AeroCoeffs[2][0])*(AoA-AeroCoeffs[2][0])+AeroCoeffs[2][1];
    CD = (AeroCoeffs[3][2]-AeroCoeffs[2][2])/(AeroCoeffs[3][0]-AeroCoeffs[2][0])*(AoA-AeroCoeffs[2][0])+AeroCoeffs[2][2];
  }else if(2<AoA && AoA<=4){
    CL = (AeroCoeffs[4][1]-AeroCoeffs[3][1])/(AeroCoeffs[4][0]-AeroCoeffs[3][0])*(AoA-AeroCoeffs[3][0])+AeroCoeffs[3][1];
    CD = (AeroCoeffs[4][2]-AeroCoeffs[3][2])/(AeroCoeffs[4][0]-AeroCoeffs[3][0])*(AoA-AeroCoeffs[3][0])+AeroCoeffs[3][2];
  }else if(4<AoA && AoA<=6){
    CL = (AeroCoeffs[5][1]-AeroCoeffs[4][1])/(AeroCoeffs[5][0]-AeroCoeffs[4][0])*(AoA-AeroCoeffs[4][0])+AeroCoeffs[4][1];
    CD = (AeroCoeffs[5][2]-AeroCoeffs[4][2])/(AeroCoeffs[5][0]-AeroCoeffs[4][0])*(AoA-AeroCoeffs[4][0])+AeroCoeffs[4][2];
  }else if(6<AoA && AoA<=8){
    CL = (AeroCoeffs[6][1]-AeroCoeffs[5][1])/(AeroCoeffs[6][0]-AeroCoeffs[5][0])*(AoA-AeroCoeffs[5][0])+AeroCoeffs[5][1];
    CD = (AeroCoeffs[6][2]-AeroCoeffs[5][2])/(AeroCoeffs[6][0]-AeroCoeffs[5][0])*(AoA-AeroCoeffs[5][0])+AeroCoeffs[5][2];
  }else if(8<AoA && AoA<=10){
    CL = (AeroCoeffs[7][1]-AeroCoeffs[6][1])/(AeroCoeffs[7][0]-AeroCoeffs[6][0])*(AoA-AeroCoeffs[6][0])+AeroCoeffs[6][1];
    CD = (AeroCoeffs[7][2]-AeroCoeffs[6][2])/(AeroCoeffs[7][0]-AeroCoeffs[6][0])*(AoA-AeroCoeffs[6][0])+AeroCoeffs[6][2];
  }else if(10<AoA && AoA<=12){
    CL = (AeroCoeffs[8][1]-AeroCoeffs[7][1])/(AeroCoeffs[8][0]-AeroCoeffs[7][0])*(AoA-AeroCoeffs[7][0])+AeroCoeffs[7][1];
    CD = (AeroCoeffs[8][2]-AeroCoeffs[7][2])/(AeroCoeffs[8][0]-AeroCoeffs[7][0])*(AoA-AeroCoeffs[7][0])+AeroCoeffs[7][2];
  }else{
    CL = 0;
    CD = 0;
  };

  float Thb[3][3] = {
    {cos(theta)*cos(psi),cos(theta)*sin(psi),-sin(theta)},
    {sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),sin(phi)*cos(theta)},
    {cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),cos(phi)*cos(theta)}
  };
  float det_Thb = Thb[0][0]*Thb[1][1]*Thb[2][2]-Thb[0][0]*Thb[1][2]*Thb[2][1]+Thb[0][1]*Thb[1][2]*Thb[2][0]-Thb[0][1]*Thb[1][0]*Thb[2][2]+Thb[0][2]*Thb[1][0]*Thb[2][1]-Thb[0][2]*Thb[1][1]*Thb[2][0];
  float Thb_I[3][3] = {
    {(Thb[1][1]*Thb[2][2]-Thb[1][2]*Thb[2][1])/det_Thb,-(Thb[0][1]*Thb[2][2]-Thb[0][2]*Thb[2][1])/det_Thb,(Thb[0][1]*Thb[1][2]-Thb[0][2]*Thb[1][1])/det_Thb},
    {-(Thb[1][0]*Thb[2][2]-Thb[1][2]*Thb[2][0])/det_Thb,(Thb[0][0]*Thb[2][2]-Thb[0][2]*Thb[2][0])/det_Thb,-(Thb[0][0]*Thb[1][2]-Thb[0][2]*Thb[1][0])/det_Thb},
    {(Thb[1][0]*Thb[2][1]-Thb[1][1]*Thb[2][0])/det_Thb,-(Thb[0][0]*Thb[2][1]-Thb[0][1]*Thb[2][0])/det_Thb,(Thb[0][0]*Thb[1][1]-Thb[0][1]*Thb[1][0])/det_Thb}
  };
  float Dw[3] = {(rho*(pow(Vc,2))*S*CL)/2,0,0};
  float Db[3]={
    Twb[0][0]*Dw[0] + Twb[0][1]*Dw[1] + Twb[0][2]*Dw[2],
    Twb[1][0]*Dw[0] + Twb[1][1]*Dw[1] + Twb[1][2]*Dw[2],
    Twb[2][0]*Dw[0] + Twb[2][1]*Dw[1] + Twb[2][2]*Dw[2]
  };

  float Mg_T = M*g*cos(fabs(phi))*cos(3.141592653589/2 - theta);
  float T_0 = -Db[0] + Mg_T;
  float Vc_tar;

  if(CL != 0 && CD != 0){
    float Vc_tar = sqrt(2*M*g/(rho*S*CL*(Thb_I[2][0]*Twb[0][2]+Thb_I[2][1]*Twb[1][2]+Thb_I[2][2]*Twb[2][2])));
  }else{
    float Vc_tar = 0;
  };

  float thrust = Kp*(Vc_tar - Vc) - Kd*(T - T_0)/M + T_0;
  return thrust;
}

void loop() {
  // put your main code here, to run repeatedly:
  noInterrupts();
  uint32_t pulseWidth_S = PW_S;
  uint32_t period_S = Period_S;
  interrupts();
    if(period_S>0){
    Duty_S = (float)pulseWidth_S/(float)period_S;
    Duty_S = constrain(Duty_S, 0, 1);
  }else{
    Duty_S=0.0;
  }

  if(Duty_S>0.9){
  //horizontal rotation
    digitalWrite(pin_num_LED_H,HIGH);

/*

    //nine axis sensor
    unsigned long currentTime=millis();
    float dt = (currentTime-previousTime)/1000.0;
    previousTime = currentTime;
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    float Vc = sqrt(pow(accel[0]*dt,2)+pow(accel[1]*dt,2)+pow(accel[2]*dt,2));
    float AoA = atan((accel[2]*dt)/(accel[0]*dt));
    float B = asin((accel[1]*dt)/Vc);

    float Twb[3][3] = {
      {cos(AoA)*cos(-B),cos(AoA)*sin(-B),-sin(AoA)},
      {-sin(-B),cos(-B),0},
      {sin(AoA)*cos(-B),sin(AoA)*sin(-B),cos(AoA)}
    };
    float det_Twb = Twb[0][0]*Twb[1][1]*Twb[2][2]-Twb[0][0]*Twb[1][2]*Twb[2][1]+Twb[0][1]*Twb[1][2]*Twb[2][0]-Twb[0][1]*Twb[1][0]*Twb[2][2]+Twb[0][2]*Twb[1][0]*Twb[2][1]-Twb[0][2]*Twb[1][1]*Twb[2][0];
    float Twb_I[3][3] = {
      {(Twb[1][1]*Twb[2][2]-Twb[1][2]*Twb[2][1])/det_Twb,-(Twb[0][1]*Twb[2][2]-Twb[0][2]*Twb[2][1])/det_Twb,(Twb[0][1]*Twb[1][2]-Twb[0][2]*Twb[1][1])/det_Twb},
      {-(Twb[1][0]*Twb[2][2]-Twb[1][2]*Twb[2][0])/det_Twb,(Twb[0][0]*Twb[2][2]-Twb[0][2]*Twb[2][0])/det_Twb,-(Twb[0][0]*Twb[1][2]-Twb[0][2]*Twb[1][0])/det_Twb},
      {(Twb[1][0]*Twb[2][1]-Twb[1][1]*Twb[2][0])/det_Twb,-(Twb[0][0]*Twb[2][1]-Twb[0][1]*Twb[2][0])/det_Twb,(Twb[0][0]*Twb[1][1]-Twb[0][1]*Twb[1][0])/det_Twb} 
    };

    float gyro_w[3]={
      Twb_I[0][0]*gyro.x() + Twb_I[0][1]*gyro.y() + Twb_I[0][2]*gyro.z(),
      Twb_I[1][0]*gyro.x() + Twb_I[1][1]*gyro.y() + Twb_I[1][2]*gyro.z(),
      Twb_I[2][0]*gyro.x() + Twb_I[2][1]*gyro.y() + Twb_I[2][2]*gyro.z()
    };

    float Q = gyro_w[1];
    float P = gyro_w[0];

    Madgwick.update(gyro.x(), gyro.y(), gyro.z(), accel.x(), accel.y(), accel.z(), mag.x(), mag.y(), mag.z());
    float theta = Madgwick.getPitch();
    float psi = Madgwick.getYaw();
    float phi = Madgwick.getRoll();

    //servo
    double de_val = fabs(de(AoA,Q));
    double da_val = fabs(da(phi,P));
    float Duty_de_val = de_val/180;
    float Duty_da_val = da_val/180;
    ledcWrite(pin_num_SVe, 255*Duty_de_val);
    ledcWrite(pin_num_SVal, 255*Duty_da_val);
    ledcWrite(pin_num_SVar, 255*Duty_da_val);

    //motor
    double T = thrust(T,Vc,AoA,B,theta,psi,phi,Twb);
    double Duty_MT_auto = (Duty_max_MT-Duty_min_MT)/(thrust_max-thrust_min)*(T-thrust_min)+Duty_min_MT;
    Duty_MT_auto = constrain(Duty_MT_auto, 0, 1);
    ledcWrite(pin_num_MT, Duty_MT_auto*255);

///////////////////////////////

Serial.print(Duty_de_val,3);
  Serial.print("\t");
Serial.print(Duty_da_val,3);
  Serial.print("\t");
Serial.print(Duty_MT_auto,3);
  Serial.print("\t");
Serial.print(phi,3);
  Serial.print("\t");

//////////////////////////////

*/

    }else{
      //proportional control
      digitalWrite(pin_num_LED_H,LOW);

    //servo
  noInterrupts();
  uint32_t pulseWidth_de = PW_de;
  uint32_t period_de = Period_de;
  interrupts();
    if(period_de>0){
    Duty_de = (float)pulseWidth_de/(float)period_de;
    Duty_de = constrain(Duty_de, 0, 1);
  }else{
    Duty_de=0.0;
  }
    ledcWrite(channel_SVe, 255*Duty_de);
  noInterrupts();
  uint32_t pulseWidth_da = PW_da;
  uint32_t period_da = Period_da;
  interrupts();
    if(period_da>0){
    Duty_da = (float)pulseWidth_da/(float)period_da;
    Duty_da = constrain(Duty_da, 0, 1);
  }else{
    Duty_da=0.0;
  }
    ledcWrite(channel_SVal, 255*Duty_da);
    ledcWrite(channel_SVar, 255*Duty_da);

    //motor
  noInterrupts();
  uint32_t pulseWidth_MT = PW_MT;
  uint32_t period_MT = Period_MT;
  interrupts();
    if(period_MT>0){
    Duty_MT = (float)pulseWidth_MT/(float)period_MT;
    Duty_MT = constrain(Duty_MT, 0, 1);
  }else{
    Duty_MT=0.0;
  }
    ledcWrite(channel_MT, Duty_MT*255);

//////////////////////////

Serial.print(Duty_de,3);
  Serial.print("\t");
Serial.print(Duty_da,3);
  Serial.print("\t");
Serial.print(Duty_MT,3);
  Serial.print("\t");

//////////////////////////

    }

/////////////////////////////
Serial.println(Duty_S,3);
//delay(100);
/////////////////////////////
}
