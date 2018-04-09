#include "Wire.h"
#include "MPU9150.h"

//Debugging purpose
#define DebugMonitor false
#define LED_PIN 13
bool blinkState = false;

//MPU9150 Variables
MPU9150 accelGyroMag;
int16_t ax, ay, az; //accelerometer
int16_t gx, gy, gz; //gyroscope
int16_t mx, my, mz; //magnetometer

//old value for filtering
int16_t o_ax, o_ay, o_az;
int16_t o_gx, o_gy, o_gz;
int16_t o_mx, o_my, o_mz;
bool changed;

//high-pass filter value
#define filter_a 300
#define filter_g 70
#define filter_m 3000


uint32_t g_2, m_2;
float g_magnitude, m_magnitude;
float gx_normal, gy_normal, gz_normal, mx_normal, my_normal, mz_normal;

//Transferring
#define PRECISION 10000
#define MASK_8bit B11111111
const int MASK_16bit = MASK_8bit << 8 | MASK_8bit;

void setup() {
  Wire.begin();
  Serial.begin(38400);
  
  if(DebugMonitor) Serial.println("Initializing I2C devices...");
  accelGyroMag.initialize();

  // verify connection
  if(DebugMonitor) Serial.println("Testing device connections...");
  if(DebugMonitor) Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");

  g_2 = 1; m_2 = 1;
  g_magnitude = 1;
  m_magnitude = 1;

  accelGyroMag.getMotion9(&o_ax, &o_ay, &o_az, &o_gx, &o_gy, &o_gz, &o_mx, &o_my, &o_mz);
  changed = false;
  
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
/*
  //Filtering Accelerometer
  if(ax - o_ax > filter_a){
    o_ax = ax;
    changed = true;  }
  if(ay - o_ay > filter_a){
    o_ay = ay;
    changed = true;  }
  if(az - o_az > filter_a){
    o_az = az;
    changed = true;  }
    
  //Filtering Gyroscope
  if(gx - o_gx > filter_g){
    o_gx = gx;
    changed = true;  }
  if(gy - o_gy > filter_g){
    o_gy = gy;
    changed = true;  }
  if(gz - o_gz > filter_g){
    o_gz = gz;
    changed = true;  }
    
  //Filtering Magnetometer
  if(mx - o_mx > filter_m){
    o_mx = mx;
    changed = true;  }
  if(my - o_my > filter_m){
    o_my = my;
    changed = true;  }
  if(mz - o_mz > filter_m){
    o_mz = mz;
    changed = true;  }

  if(changed){*/
   /* g_2 = (uint32_t)gx * gx + (uint32_t)gy * gy + (uint32_t)gz * gz;
    g_magnitude = sqrt(g_2);
    gx_normal = (float)gx / g_magnitude;
    gy_normal = (float)gy / g_magnitude;
    gz_normal = (float)gz / g_magnitude;*/
  
    m_2 = (uint32_t)mx * mx + (uint32_t)my * my + (uint32_t)mz * mz;
    m_magnitude = sqrt(m_2);
    mx_normal = (float)mx / m_magnitude;
    my_normal = (float)my / m_magnitude;
    mz_normal = (float)mz / m_magnitude;
 // }
  
  if(DebugMonitor){
    Serial.print("g:\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");
    Serial.print("m:\t");
    Serial.print(mx); Serial.print("\t");
    Serial.print(my); Serial.print("\t");
    Serial.print(mz); Serial.print("\t");
    Serial.print("m_normal:\t");
    Serial.print(mx_normal,4); Serial.print("\t");
    Serial.print(my_normal,4); Serial.print("\t");
    Serial.print(mz_normal,4); Serial.print("\t");
    Serial.print(m_2); Serial.print("\t");
    Serial.print(m_magnitude); Serial.print("\t");
    Serial.print("\n");
  } else {
    if (Serial.available()) {
      if (Serial.read() == 's') {
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        Serial.write(0xff) ;
        if(PRECISION > 10000) Serial.write(0x44);
        else Serial.write(0x22);
        /*sendFloatWithPrecision(gx_normal, PRECISION);
        sendFloatWithPrecision(gy_normal, PRECISION);
        sendFloatWithPrecision(gz_normal, PRECISION);*/
        sendFloatWithPrecision(mx_normal, PRECISION);
        sendFloatWithPrecision(my_normal, PRECISION);
        sendFloatWithPrecision(mz_normal, PRECISION);
        delay(50);
      }
    }
    
  }
  changed = false;
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
