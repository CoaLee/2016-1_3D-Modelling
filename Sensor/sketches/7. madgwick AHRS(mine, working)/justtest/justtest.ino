#include "Wire.h"
#include "MPU9150.h"

//MPU9150 Variables
MPU9150 accelGyroMag;
int16_t ax, ay, az; //accelerometer
int16_t gx, gy, gz; //gyroscope
int16_t mx, my, mz; //magnetometer

//high-pass filter value
#define filter_a 300
#define filter_g 70
#define filter_m 3000

//Transferring
#define PRECISION 10000
#define MASK_8bit B11111111
const int MASK_16bit = MASK_8bit << 8 | MASK_8bit;

//Debugging purpose
#define DebugMonitor true
#define LED_PIN 13
bool blinkState = false;


void setup() {
  Wire.begin();
  Serial.begin(38400);
  
  if(DebugMonitor) Serial.println("Initializing I2C devices...");
  accelGyroMag.initialize();

  // verify connection
  if(DebugMonitor) Serial.println("Testing device connections...");
  if(DebugMonitor) Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  if(DebugMonitor){
    Serial.print("g:\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");
    Serial.print("m:\t");
    Serial.print(mx); Serial.print("\t");
    Serial.print(my); Serial.print("\t");
    Serial.print(mz); Serial.print("\t");
    
    Serial.print("\n");
    delay(100);
  } else {
    
  }
  
}

// Fast inverse square-root for float
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x){
   uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
   float tmp = *(float*)&i;
   return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}

//Experimental
void sendFloatWithPrecision(float arg, int precision){
  if(precision>10000) 
    sendFloatin32Bit(arg, precision);
  else 
    sendFloatin16Bit(arg, precision);
}

void sendFloatin16Bit(float arg, int precision){
  uint8_t sign = 3;
  int16_t num = arg * precision;
  if(num < 0) {
    sign -= 2;
    num *= -1; 
  }
  uint8_t first_half = num>>8;
  uint8_t second_half = num & MASK_8bit;

  if(DebugMonitor){
    if(sign==1) Serial.print("-");
    else Serial.print("+");
    Serial.print(first_half);
    Serial.print(second_half);
    Serial.print("\n");
    Serial.flush();   
  } else {
    Serial.write(sign);
    Serial.write(first_half);
    Serial.write(second_half);
    Serial.flush();
  }
}

void sendFloatin32Bit(float arg, int precision){
  uint8_t sign = 3;
  int32_t num = arg * precision;
  uint16_t first_half = num>>16;
  uint16_t second_half = num & MASK_16bit;
  uint8_t first_quarter = first_half >> 8;
  uint8_t second_quarter = first_half & MASK_8bit;
  uint8_t third_quarter = second_half >> 8;
  uint8_t fourth_quarter = second_half & MASK_8bit;

  if(num < 0) sign -= 2;

  if(DebugMonitor){
    if(sign==1) Serial.print("-");
    else Serial.print("+");
    Serial.print(first_quarter);
    Serial.print(second_quarter);
    Serial.print(third_quarter);
    Serial.print(fourth_quarter);
    Serial.print("\n");
    Serial.flush();
  } else {
    Serial.write(sign);
    Serial.write(first_quarter);
    Serial.write(second_quarter);
    Serial.write(third_quarter);
    Serial.write(fourth_quarter);
    Serial.flush();
  }

}
