/* Sensor Experiment:
 *  0829) baud_rate 38400
 */
#include "Wire.h"
#include "MPU9150.h"

//MPU9150 Variables
MPU9150 MPU;
int16_t ax, ay, az; //accelerometer
int16_t gx, gy, gz; //gyroscope
int16_t mx, my, mz; //magnetometer

//Debugging purpose
#define DebugMonitor true
#define LED_PIN 13
bool blinkState = false;

//testing
int32_t sum_ax, sum_ay, sum_az, sum_gx, sum_gy, sum_gz;
int32_t cnt;
int32_t avg_ax, avg_ay, avg_az, avg_gx, avg_gy, avg_gz;


void setup() {
  Wire.begin();

  Serial.begin(115200);
  
  if(DebugMonitor) Serial.println("Initializing I2C devices...");
  MPU.initialize();

  // verify connection
  if(DebugMonitor) Serial.println("Testing device connections...");
  if(DebugMonitor) Serial.println(MPU.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");

//testing
  MPU.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);


  MPU.setXGyroOffset(36);
  MPU.setYGyroOffset(37);
  MPU.setZGyroOffset(-16);
  //MPU.setZAccelOffset(-200);
  
  sum_ax = 0; sum_ay = 0; sum_az = 0; 
  cnt = 0;
  sum_gx = 0; sum_gy = 0; sum_gz = 0; 
 
  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  MPU.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    
  cnt++;
  sum_ax += ax; sum_ay += ay; sum_az += az;
  sum_gx += gx; sum_gy += gy; sum_gz += gz;

  avg_ax = sum_ax / cnt; avg_ay = sum_ay / cnt; avg_az = sum_az / cnt;
  avg_gx = sum_gx / cnt; avg_gy = sum_gy / cnt; avg_gz = sum_gz / cnt;

  if(DebugMonitor){ 
    Serial.print("cnt: "); Serial.print(cnt); Serial.print("\t");
    Serial.print("a:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print("g:\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");
    Serial.print("sum_a:\t");
    Serial.print(sum_ax); Serial.print("\t");
    Serial.print(sum_ay); Serial.print("\t");
    Serial.print(sum_az); Serial.print("\t");
    Serial.print("avg_a:\t");
    Serial.print(avg_ax); Serial.print("\t");
    Serial.print(avg_ay); Serial.print("\t");
    Serial.print(avg_az); Serial.print("\t");
    Serial.print("sum_g:\t");
    Serial.print(sum_gx); Serial.print("\t");
    Serial.print(sum_gy); Serial.print("\t");
    Serial.print(sum_gz); Serial.print("\t");
    Serial.print("avg_g:\t");
    Serial.print(avg_gx); Serial.print("\t");
    Serial.print(avg_gy); Serial.print("\t");
    Serial.print(avg_gz); Serial.print("\t");
    /*Serial.print("m:\t");
    Serial.print(mx); Serial.print("\t");
    Serial.print(my); Serial.print("\t");
    Serial.print(mz); Serial.print("\t");
*/
    Serial.print("\n");
  }
}
