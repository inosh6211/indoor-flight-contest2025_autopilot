#include<Servo.h>
#include<Wire.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BNO055.h>
#include<MadgwickAHRS.h>

//nine axis sensor
  Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28);
  Madgwick Madgwick;
  unsigned long previousTime=0;

void setup() {
  // put your setup code here, to run once:


//nine axis sensor
  Serial.begin(115200);
  Madgwick.begin(100.0f);
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring!");
    while (1);
}
}

void loop() {
  // put your main code here, to run repeatedly:

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

Serial.print(Vc,3);
  Serial.print("\t");
Serial.print(AoA,3);
  Serial.print("\t");
Serial.print(B,3);
  Serial.print("\t");
Serial.print(P,3);
  Serial.print("\t");
Serial.print(Q,3);
  Serial.print("\t");
Serial.print(theta,3);
  Serial.print("\t");
Serial.print(psi,3);
  Serial.print("\t");
Serial.println(phi,3);
delay(50);
}
