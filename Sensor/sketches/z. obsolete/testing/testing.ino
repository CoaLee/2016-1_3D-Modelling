#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9150.h"
#include "helper_3dmath.h"

MPU9150 accelGyroMag;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

#define LED 13

bool switch_led = false;
//bool going = false;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  

  accelGyroMag.initialize();
  
  pinMode(LED, OUTPUT);
  
}

void loop() {
    accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    
  if(Serial.available()>0){
        Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
    Serial.print(",");
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.print(gz);
    Serial.print(",");
    Serial.print(mx);
    Serial.print(",");
    Serial.print(my);
    Serial.print(",");
    Serial.print(mz);
    Serial.print("\n");
    digitalWrite(LED, switch_led);
    switch_led = !switch_led;
  }
    
  delay(30);
}

