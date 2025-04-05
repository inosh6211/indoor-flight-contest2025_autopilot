#include<Servo.h>
#include<Wire.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BNO055.h>
#include<MadgwickAHRS.h>

//servo
Servo SVal;
Servo SVar;
int pin_num_SVal = 9;
int pin_num_SVar = 3;
int deg_min_sg90 = 0;
int deg_max_sg90 = 180;

//switch of autopilot
int pin_num_switch_h = 2;
volatile uint32_t startTime_S=0;
volatile uint32_t PW_S=0;
volatile uint32_t Period_S=0;
volatile float Duty_S=0;

//LED
int pin_num_LED_H = 6;

//nine axis sensor
  Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28);
  Madgwick Madgwick;
  unsigned long previousTime=0;

void setup() {
  // put your setup code here, to run once:
//servo
  SVal.attach(pin_num_SVal);
  SVar.attach(pin_num_SVar);

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
  static uint32_t endTime_S=0;
  uint32_t currentTime=micros();
  if(digitalRead(pin_num_switch_h)==HIGH){
    startTime_S = currentTime;
    Period_S = currentTime-endTime_S;
    endTime_S = currentTime;
  }else{
    PW_S = currentTime-startTime_S;
  }

  if(Period_S>0){
    Duty_S=(double)PW_S/(double)Period_S;
    Duty_S = constrain(Duty_S, 0, 1);
  }else{
    Duty_S=0.0;
  }
}

float da(float phi,float P){//control law of aileron
  float Kp=1;
  float Kd=1;
  float phi_tar=8;

  float da=Kp*(phi_tar-phi)-Kd*P;
  return da;
}

void loop() {
  // put your main code here, to run repeatedly:

  if(Duty_S>0.12){
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
    double da_val = fabs(da(phi,P));
float Duty_da_val = da_val/180;
analogWrite(pin_num_SVal,255*Duty_da_val);

Serial.print(Duty_da_val,3);
 Serial.print("\t");
Serial.print(da_val,3);
 Serial.print("\t");
Serial.print(Duty_S,3);
 Serial.print("\t");
Serial.println(phi,3);

delay(200);
  }else{
  //proportional control
      digitalWrite(pin_num_LED_H,LOW);
  }
}
