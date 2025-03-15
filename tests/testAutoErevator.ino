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

float de(float AoA,float Q){//control law of elevator
  float Kp=1;
  float Kd=1;
  float AoA_tar=0;
  float Cm_tar=0.00961;
  float Cm_de_tar=0.327;
  float de_0=Cm_tar/Cm_de_tar;

  float de=Kp*(AoA_tar-AoA)-Kd*Q+de_0;
  return de;
}

void loop() {
  // put your main code here, to run repeatedly:
  int state_switch_h = digitalRead(pin_num_switch_h);
  int state_switch_8 = digitalRead(pin_num_switch_8);

  if(state_switch_h == HIGH){
  //horizontal rotation
    digitalWrite(pin_num_LED_H,HIGH);
    digitalWrite(pin_num_LED_L,LOW);

    //nine axis sensor
    unsigned long currentTime=millis();
    float dt = (currentTime-previousTime)/1000.0;
    previousTime = currentTime;
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    imu::Vector<3> Vb = accel*dt;
    float Vc = pow(pow(Vb[0],2)+pow(Vb[1],2)+pow(Vb[2],2),1/2);

    float AoA = atan(Vb[2]/Vb[0]);
    float B = asin(Vb[1]/Vc);

  float Twb[3][3] = {
    {cos(AoA)*cos(-B),cos(AoA)*sin(-B),-sin(AoA)},
    {-sin(-B),cos(-B),0},
    {sin(AoA)*cos(-B),sin(AoA)*sin(-B),cos(AoA)}
  };
  float det_Twb = Twb[0][0]*Twb[1][1]*Twb[2][2]-Twb[0][0]*Twb[1][2]*Twb[2][1]+Twb[0][1]*Twb[1][2]*Twb[2][0]-Twb[0][1]*Twb[1][0]*Twb[2][2]+Twb[0][2]*Twb[1][0]*Twb[2][1]-Twb[0][2]*Twb[1][1]*Twb[2][0];
  float Twb_I[3][3] = {
    {Twb[1][1]*Twb[2][2]-Twb[1][2]*Twb[2][1],-(Twb[0][1]*Twb[2][2]-Twb[0][2]*Twb[2][1]),Twb[0][1]*Twb[1][2]-Twb[0][2]*Twb[1][1]},
    {-(Twb[1][0]*Twb[2][2]-Twb[1][2]*Twb[2][0]),Twb[0][0]*Twb[2][2]-Twb[0][2]*Twb[2][0],-(Twb[0][0]*Twb[1][2]-Twb[0][2]*Twb[1][0])},
    {Twb[1][0]*Twb[2][1]-Twb[1][1]*Twb[2][0],-(Twb[0][0]*Twb[2][1]-Twb[0][1]*Twb[2][0]),Twb[0][0]*Twb[1][1]-Twb[0][1]*Twb[1][0]}
  }/det_Twb;

  imu::Vector<3> gyro_w = Twb_I*gyro;

    float Q = gyro_w[1];
    float P = gyro_w[0];

    Madgwick.update(gyro.x(),gyro.y(),gyro.z(),accel.x(),accel.y(),accel.z(),mag.x(),mag.y(),mag.z());
    float qw = Madgwick.getQuatW();
    float qx = Madgwick.getQuatX();
    float qy = Madgwick.getQuatY();
    float qz = Madgwick.getQuatZ();

    float theta = asin(2.0*(qw*qy-qz*qx))*180.0/PI;
    float psi = atan2(2.0*(qw*qz+qx*qy),1.0-2.0*(qy*qy+qz*qz))*180.0/PI;
    float phi = atan2(2.0*(qw*qx+qy*qz),1.0-2.0*(qx*qx+qy*qy))*180.0/PI;

    //servo
    double de = de(AoA,Q);
    SVe.write(de);


  }else if(state_switch_8 == HIGH){
  //8 rotation
    digitalWrite(pin_num_LED_H,HIGH);
    digitalWrite(pin_num_LED_L,LOW);

////////////////////////////////////////////////////////////////////

  }else{
  }
}
