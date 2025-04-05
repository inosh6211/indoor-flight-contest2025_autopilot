#include<Servo.h>
#include<Wire.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BNO055.h>
#include<MadgwickAHRS.h>

//servo
Servo SVe;
int pin_num_SVe = 1;
int deg_min_sg90 = 0;
int deg_max_sg90 = 180;

//switch of autopilot
int pin_num_switch_h = 2;
volatile float startTime_S=0;
volatile float PW_S=0;
volatile float Period_S=0;
volatile float Duty_S=0;

//LED
int pin_num_LED_H = 6;

//nine axis sensor
  Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28);
  Madgwick Madgwick;
  unsigned long previousTime=0;

//reciever


void setup() {
  // put your setup code here, to run once:
//servo
  SVe.attach(pin_num_SVe);

//reciever


//switch of autopilot
  pinMode(pin_num_switch_h,INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_num_switch_h),pulseISR_S,CHANGE);

//nine axis sensor
  Serial.begin(115200);
  Madgwick.begin(100.0f);
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring!");
    while (1);
}

//LED
  pinMode(pin_num_LED_H,OUTPUT);
}

void pulseISR_S(){
  static unsigned long endTime_S=0;
  unsigned long currentTime=micros();
  if(digitalRead(pin_num_switch_h)==HIGH){
    startTime_S = currentTime;
    Period_S = currentTime-endTime_S;
    endTime_S = currentTime;
  }else{
    PW_S = micros()-startTime_S;
  }

  if(Period_S>0){
    Duty_S=(float)PW_S/(float)Period_S;
  }else{
    Duty_S=0.0;
  }
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

  if(Duty_S>0.1){
  //horizontal rotation
    digitalWrite(pin_num_LED_H,HIGH);

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
    {(Twb[1][0]*Twb[2][1]-Twb[1][1]*Twb[2][0])/det_Twb,-(Twb[0][0]*Twb[2][1]-Twb[0][1]*Twb[2][0])/det_Twb,(Twb[0][0]*Twb[1][1]-Twb[0][1]*Twb[1][0])/det_Twb} };

  //imu::Vector<3> gyro_w = Twb_I*gyro;
float gyro_w[3]={
    Twb_I[0][0] * gyro.x() + Twb_I[0][1] * gyro.y() + Twb_I[0][2] * gyro.z(),
    Twb_I[1][0] * gyro.x() + Twb_I[1][1] * gyro.y() + Twb_I[1][2] * gyro.z(),
    Twb_I[2][0] * gyro.x() + Twb_I[2][1] * gyro.y() + Twb_I[2][2] * gyro.z()
};

    float Q = gyro_w[1];
    float P = gyro_w[0];

    Madgwick.update(gyro.x(),gyro.y(),gyro.z(),accel.x(),accel.y(),accel.z(),mag.x(),mag.y(),mag.z());

float theta = Madgwick.getPitch();  
float psi = Madgwick.getYaw();   
float phi = Madgwick.getRoll();  

    //servo
    float de_val = de(AoA,Q);
    SVe.write(de_val);

Serial.print(de_val,3);
 Serial.print("\t");
Serial.println(AoA,3);

delay(200);
  }else{
  //proportional control
      digitalWrite(pin_num_LED_H,LOW);
  }
}
