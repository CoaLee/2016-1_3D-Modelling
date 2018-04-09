#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9150.h"
#define pi 3.141592
MPU9150 accelGyroMag;

int i;
double px, py, pz, qx, qy, qz;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

double myaw, apit, arol;
double yaw, pitch, roll;
double gyaw = 0, gpit = 0, grol = 0, pre_gyaw, pre_gpit, pre_grol;
unsigned long used_millisec, pre_millisec = 0;
double Co_gain = 1;///change

void setup() {
    
    Wire.begin();
    Serial.begin(115200);//must change
    
    accelGyroMag.initialize();
    Serial.print("initialize sucess ");
    Serial.print("               ");
   // delay(1000);
    Serial.print("calibration....");
    Serial.print("               ");
   // delay(1500);
    Serial.print("waiting....");
    
  for(i = 1 ; i <= 500 ; i++){
    accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    px = 0 - gx;
    py = 0 - gy;
    pz = 0 - gz;
    qx = qx * (i-1) / i + (px/i);
    qy = qy * (i-1) / i + (py/i);
    qz = qz * (i-1) / i + (pz/i);
  }
    
  yaw = atan2(my, mx)*180/pi;
  pitch = atan2(az, ax)*180/pi;
  roll = atan2(az, ay)*180/pi;

  Serial.println("ready...");
  delay(2000);
  Serial.println("GO");
}

void loop() {
  used_millisec = millis() - pre_millisec;
  pre_millisec = millis();
  
  gyaw = pre_gyaw * used_millisec / 1000;
  gpit = pre_gpit * used_millisec / 1000;
  grol = pre_grol * used_millisec / 1000;

  
  yaw += gyaw * (Co_gain - 1)/Co_gain + (myaw - yaw)/Co_gain;
  pitch += gpit * (Co_gain - 1)/Co_gain + (apit - pitch)/Co_gain;
  roll += grol * (Co_gain - 1)/Co_gain + (arol - roll)/Co_gain;
  
  accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  myaw = atan2(my, mx)*180/pi;
  apit = atan2(az, ax)*180/pi;
  arol = atan2(az, ay)*180/pi;

  pre_gyaw = mz + qz;
  pre_gpit = my + qy;
  pre_grol = mx + qx;
  
  Serial.print("yaw : ");
  Serial.print(yaw);    
  Serial.print(" ");
  Serial.print("pit : ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print("rol : ");
  Serial.println(roll);
}
