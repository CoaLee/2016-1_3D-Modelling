#include "Wire.h"
#include "MPU9150.h"

#define mpu_add 0x68
#define DebugMonitor1 true
#define DebugMonitor2 true

MPU9150 MPU;
int16_t ax, ay, az, gx, gy, gz;

double deg[3], dgx, dgy, dgz ;
double dt ;
uint32_t pasttime ;

//Transferring
#define PRECISION 10000
#define MASK_8bit B11111111
const int MASK_16bit = MASK_8bit << 8 | MASK_8bit;

class kalman {
  public :
    double getkalman(double acc, double gyro, double dt) {
      //project the state ahead
      angle += dt * (gyro - bias) ;

      //Project the error covariance ahead
      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle) ;
      P[0][1] -= dt * P[1][1] ;
      P[1][0] -= dt * P[1][1] ;
      P[1][1] += Q_gyro * dt ;

      //Compute the Kalman gain
      double S = P[0][0] + R_measure ;
      K[0] = P[0][0] / S ;
      K[1] = P[1][0] / S ;

      //Update estimate with measurement z
      double y = acc - angle ;
      angle += K[0] * y ;
      bias += K[1] * y ;

      //Update the error covariance
      double P_temp[2] = {P[0][0], P[0][1]} ;
      P[0][0] -= K[0] * P_temp[0] ;
      P[0][1] -= K[0] * P_temp[1] ;
      P[1][0] -= K[1] * P_temp[0] ;
      P[1][1] -= K[1] * P_temp[1] ;

      return angle ;
    } ;
    
    void init(double angle, double gyro, double measure) {
      Q_angle = angle ;
      Q_gyro = gyro ;
      R_measure = measure ;

      angle = 0 ;
      bias = 0 ;

      P[0][0] = 0 ;
      P[0][1] = 0 ;
      P[1][0] = 0 ;
      P[1][1] = 0 ;
    } ;
    
    double getvar(int num) {
      switch (num) {
        case 0 :
          return Q_angle ;
          break ;
        case 1 :
          return Q_gyro ;
          break ;
        case 2 :
          return R_measure ;
          break ;
      }
    } ;
    
  private :
    double Q_angle, Q_gyro, R_measure ;
    double angle, bias ;
    double P[2][2], K[2] ;
} ;

kalman kalx ;
kalman kaly ;
kalman kalz ;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600) ;
  
  Wire.begin() ;
  Wire.beginTransmission(mpu_add) ;
  Wire.write(0x6B) ;
  Wire.write(0) ;
  Wire.endTransmission(true) ;

  MPU.setRate(7);
  
  /* Digital low pass filter configuration. 
   * It also determines the internal sampling rate used by the device as shown in the table below.
   * The accelerometer output rate is fixed at 1kHz. This means that for a Sample
   * Rate greater than 1kHz, the same accelerometer sample may be output to the
   * FIFO, DMP, and sensor registers more than once.
   * 
   *          |   ACCELEROMETER    |           GYROSCOPE
   * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
   * ---------+-----------+--------+-----------+--------+-------------
   * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
   * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
   * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
   * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
   * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
   * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
   * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
  MPU.setDLPFMode(2);
  
  /* Full-scale accelerometer range.
   * The full-scale range of the accelerometer: 0 = +/- 2g, 1 = +/- 4g, 2 = +/- 8g, 3 = +/- 16g
   * */
  MPU.setFullScaleAccelRange(0);
  
  kalx.init(0.001, 0.003, 0.03) ;  //init kalman filter
  kaly.init(0.001, 0.003, 0.03) ;
  kalz.init(0.001, 0.003, 0.03) ;
}

void loop() {
  MPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  /*Wire.beginTransmission(mpu_add) ; //get acc data
  Wire.write(0x3B) ;
  Wire.endTransmission(false) ;
  Wire.requestFrom(mpu_add, 6, true) ;
  ax = Wire.read() << 8 | Wire.read() ;
  ay = Wire.read() << 8 | Wire.read() ;
  az = Wire.read() << 8 | Wire.read() ;

  Wire.beginTransmission(mpu_add) ; //get gyro data
  Wire.write(0x43) ;
  Wire.endTransmission(false) ;
  Wire.requestFrom(mpu_add, 6, true) ;
  gx = Wire.read() << 8 | Wire.read() ;
  gy = Wire.read() << 8 | Wire.read() ;
  gz = Wire.read() << 8 | Wire.read() ;*/

  deg[0] = atan2(ax, az) * 180 / PI ;  //acc data to degree data
  deg[1] = atan2(ax, ay) * 180 / PI ;
  deg[2] = atan2(ay, az) * 180 / PI ;
  dgx = gx / 131. ;  //gyro output to
  dgy = gy / 131. ;
  dgz = gz / 131. ;

  dt = (double)(millis() - pasttime) / 1000;
  pasttime = millis();  //convert output to understandable data

  double val[3] ;

  val[0] = kalx.getkalman(deg[0], dgy, dt) ;  //get kalman data
  val[1] = kaly.getkalman(deg[1], dgz, dt) ;
  val[2] = kalz.getkalman(deg[2], dgx, dt) ;

  float cax, cay, caz;

  cax = ax * 2.0f/32768.0f;
  cay = ay * 2.0f/32768.0f;
  caz = az * 2.0f/32768.0f;

  float gra_x, gra_y, gra_z;

  gra_y = sin(val[2] * (PI / 180));
  gra_x = cos(val[2] * (PI / 180)) * sin(val[0] * (PI / 180));
  gra_z = cos(val[2] * (PI / 180)) * cos(val[0] * (PI / 180));  

  float di_x, di_y, di_z;
  
  di_x = cax - gra_x;
  di_y = cay - gra_y;
  di_z = caz - gra_z;

  if (DebugMonitor1){
    Serial.print("ca_ :\t");
    Serial.print(cax); Serial.print("\t");
    Serial.print(cay); Serial.print("\t");
    Serial.print(caz); Serial.print("\t");
    Serial.print(cax*cax + cay*cay + caz*caz); Serial.print("\t");
    Serial.print("gra_ :\t");
    Serial.print(gra_x); Serial.print("\t");
    Serial.print(gra_y); Serial.print("\t");
    Serial.print(gra_z); Serial.print("\t");
    Serial.print("d_ :\t");
    Serial.print(di_x,4); Serial.print("\t");
    Serial.print(di_y,4); Serial.print("\t");
    Serial.print(di_z,4); Serial.print("\t");
  }
  
  /*a_2 = (uint32_t)ax * ax + (uint32_t)ay * ay + (uint32_t)az * az;
  a_magnitude = sqrt(a_2);
  ax_normal = (float)ax / a_magnitude;
  ay_normal = (float)ay / a_magnitude;
  az_normal = (float)az / a_magnitude;*/

  int pass_data[3] ;
  char sign[3] = {3, 3, 3} ;

  pass_data[0] = abs(val[0]) / 1 ;  //double to integer
  if (val[0] < 0) sign[0] -= 2 ;  //check sign
  pass_data[1] = abs(val[1]) / 1 ;
  if (val[1] < 0) sign[1] -= 2 ;
  pass_data[2] = abs(val[2]) / 1 ;
  if (!(val[2] < 0)) sign[2] -= 2 ;

  double dt2 = dt * dt;
  
  di_x*=dt2;
  di_y*=dt2;
  di_z*=dt2;

  if(DebugMonitor2){
    Serial.print("roll:"); Serial.print("\t");
    if(sign[0]==1) Serial.print("-"); if(sign[0]==3) Serial.print("+");
    Serial.print(pass_data[0]); Serial.print("\t");
    Serial.print("yaw:"); Serial.print("\t");
    if(sign[1]==1) Serial.print("-"); if(sign[1]==3) Serial.print("+");
    Serial.print(pass_data[1]); Serial.print("\t");
    Serial.print("pitch:"); Serial.print("\t");
    if(sign[2]==1) Serial.print("-"); if(sign[2]==3) Serial.print("+");
    Serial.print(pass_data[2]); Serial.print("\t");
    /*Serial.print("accel:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print("dt&di_:\t");
    Serial.print(dt2, 8); Serial.print("\t");*/
    Serial.print(di_x*10000,4); Serial.print("\t");
    Serial.print(di_y*10000,4); Serial.print("\t");
    Serial.print(di_z*10000,4); Serial.print("\t");

    Serial.print("\n");
  } else if(!DebugMonitor1) {
    if (Serial.available()) {
      if (Serial.read() == 's') {
        Serial.write(0xff) ;
        Serial.write(pass_data[0]) ;
        Serial.write(siagn[0]) ;
        Serial.write(pass_data[1]) ;
        Serial.write(sign[1]) ;
        Serial.write(pass_data[2]) ;
        Serial.write(sign[2]) ;
        Serial.flush() ;
        sendFloatWithPrecision(di_x*10000, PRECISION);
        sendFloatWithPrecision(di_y*10000, PRECISION);
        sendFloatWithPrecision(di_z*10000, PRECISION);
        /*sendFloatWithPrecision(ax_normal, PRECISION);
        sendFloatWithPrecision(ay_normal, PRECISION);
        sendFloatWithPrecision(az_normal, PRECISION);*/
      }
    }
  }
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

  Serial.write(sign);
  Serial.write(first_half);
  Serial.write(second_half);
  Serial.flush();
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

  Serial.write(sign);
  Serial.write(first_quarter);
  Serial.write(second_quarter);
  Serial.write(third_quarter);
  Serial.write(fourth_quarter);
  Serial.flush();
}
