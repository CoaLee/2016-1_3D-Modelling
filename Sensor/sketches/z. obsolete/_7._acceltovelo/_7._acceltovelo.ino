#include "Wire.h"
#include "MPU9150.h"

MPU9150 accelGyroMag;

int16_t ax, ay, az; //accelerometer
int16_t gx, gy, gz; //gyroscope
int16_t mx, my, mz; //magnetometer

#define LED_PIN 13
#define DebugMonitor false
bool blinkState = false;

double timenow;
double timeold;
double dt;

void setup() {
  Wire.begin();
  
  Serial.begin(9600);
  
  if(DebugMonitor) Serial.println("Initializing I2C devices...");
  accelGyroMag.initialize();

  // verify connection
  if(DebugMonitor) Serial.println("Testing device connections...");
  if(DebugMonitor) Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");

  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);

  timenow = millis();
  timeold = timenow;
  
}

void loop() {
  accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  timenow = millis();
  dt = (timenow - timeold)/1000;
  timeold = timenow;

  double vx = ax * dt;
  double vy = ay * dt;
  double vz = az * dt;
  double dx = vx * dt;
  double dy = vy * dt;
  double dz = vz * dt;

  if(DebugMonitor){
    Serial.print("a:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t| g:  ");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t| v:  ");
    Serial.print(vx); Serial.print("\t");
    Serial.print(vy); Serial.print("\t");
    Serial.print(vz); Serial.print("\t| d:  ");
    Serial.print(dx); Serial.print("\t");
    Serial.print(dy); Serial.print("\t");
    Serial.print(dz); Serial.print("\t");
    Serial.print("dt   :"); Serial.print(dt); 
    Serial.print("\n");
  }  

  int val[3];
  int pass_data[3] ;
  char sign[3] = {3, 3, 3} ;

  val[0] = dx;
  val[1] = dy;
  val[2] = dz;
  
  pass_data[0] = abs(val[0]) / 1 ;  //double to integer
  if (val[0] < 0) sign[0] -= 2 ;  //check sign

  pass_data[1] = abs(val[1]) / 1 ;
  if (val[1] < 0) sign[1] -= 2 ;

  pass_data[2] = abs(val[2]) / 1 ;
  if (!(val[2] < 0)) sign[2] -= 2 ;

  if(!DebugMonitor){
    if (Serial.available()) {
      if (Serial.read() == 's') {
        blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
        Serial.write(0xff) ;
        Serial.print(pass_data[0]) ;
        Serial.print(sign[0]) ;
        Serial.print(pass_data[1]) ;
        Serial.print(sign[1]) ;
        Serial.print(pass_data[2]) ;
        Serial.print(sign[2]) ;
        Serial.flush();
      }
    }
  }
}
