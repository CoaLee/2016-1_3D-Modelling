#define DebugMonitor true //If you want to check values through serial monitor, set it true. Current baud rate: 38400
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL
#define OUTPUT_READABLE_WORLDACCEL

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <Filters.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU9150_9Axis_MotionApps41.h"
//#include "MPU9150.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU9150 mpu;
//MPU9150 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container (float)
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//Integratation
unsigned long oldt;
double dt;
VectorFloat velWorld;   // [x, y, z]            world-frame velocity integrated from aaWorld
VectorFloat movWorld;   // [x, y, z]            world-frame movement integrated from velWorld

//Transferring
#define MASK_8bit B11111111
const int MASK_16bit = MASK_8bit << 8 | MASK_8bit;

//Filtering TOCHECK
float highFreq = 0.1f;
FilterOnePole HPF_velx(HIGHPASS, highFreq);
FilterOnePole HPF_vely(HIGHPASS, highFreq);
FilterOnePole HPF_velz(HIGHPASS, highFreq);
FilterOnePole HPF_movx(HIGHPASS, highFreq);
FilterOnePole HPF_movy(HIGHPASS, highFreq);
FilterOnePole HPF_movz(HIGHPASS, highFreq);
float lowFreq = 100.0f;
FilterOnePole LPF_accx(LOWPASS, lowFreq);
FilterOnePole LPF_accy(LOWPASS, lowFreq);
FilterOnePole LPF_accz(LOWPASS, lowFreq);
FilterOnePole LPF_movx(LOWPASS, lowFreq);
FilterOnePole LPF_movy(LOWPASS, lowFreq);
FilterOnePole LPF_movz(LOWPASS, lowFreq);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(38400);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  if(DebugMonitor) Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  if(DebugMonitor){
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  }

  // wait for ready
  if(DebugMonitor) Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  if(DebugMonitor) Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Full-scale accelerometer range.
   * The full-scale range of the accelerometer: 0 = +/- 2g, 1 = +/- 4g, 2 = +/- 8g, 3 = +/- 16g
   * */
  mpu.setFullScaleAccelRange(1);
  
  // supply your own gyro offsets here, scaled for min sensitivity
  // I think it's changing. 08202254: 36, 37, -16 working  TOCHECK
  mpu.setXGyroOffset(32);
  mpu.setYGyroOffset(48);
  mpu.setZGyroOffset(-22);
  /*mpu.setXAccelOffset();
  mpu.setYAccelOffset();
  mpu.setZAccelOffset(); */ // 1688 factory default for my test chip

  /** Set the high-pass filter configuration.
   * The DHPF is a filter module in the path leading to motion detectors (Free
   * Fall, Motion threshold, and Zero Motion). The high pass filter output is not
   * available to the data registers (see Figure in Section 8 of the MPU-6000/
   * MPU-9150 Product Specification document).
   *
   * The high pass filter has three modes:
   *    Reset: The filter output settles to zero within one sample. This
   *           effectively disables the high pass filter. This mode may be toggled
   *           to quickly settle the filter.
   *
   *    On:    The high pass filter will pass signals above the cut off frequency.
   *
   *    Hold:  When triggered, the filter holds the present sample. The filter
   *           output will be the difference between the input sample and the held
   *           sample.
   * ACCEL_HPF | Filter Mode | Cut-off Frequency
   * ----------+-------------+------------------
   * 0         | Reset       | None
   * 1         | On          | 5Hz
   * 2         | On          | 2.5Hz
   * 3         | On          | 1.25Hz
   * 4         | On          | 0.63Hz
   * 7         | Hold        | None
   * */
  //mpu.setDHPFMode(4);

  /** Set digital low-pass filter configuration.
   * The DLPF_CFG parameter sets the digital low pass filter configuration. It
   * also determines the internal sampling rate used by the device as shown in
   * the table below.
   *
   * Note: The accelerometer output rate is 1kHz. This means that for a Sample
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
   * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
   * */
  //mpu.setDLPFMode(3);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    if(DebugMonitor) Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    if(DebugMonitor) Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    if(DebugMonitor) Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    if(DebugMonitor) {
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
  }

  oldt = millis();
  
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //TOFIX if(DebugMonitor) Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    /* Integrate accel to velocity, velocity to movement */ //TOCHECK: check the calculation & filter application
    dt = (millis() - oldt) * 1e-3;
    //dt = millis() - oldt;
    oldt = millis();

    velWorld.x += HPF_velx.input((LPF_accx.input((float)aaWorld.x)) / 4096.0f * dt);
    velWorld.y += HPF_vely.input((LPF_accy.input((float)aaWorld.y)) / 4096.0f * dt);
    velWorld.z += HPF_velz.input((LPF_accz.input((float)aaWorld.z)) / 4096.0f * dt);

/*
    //Complementary Filter
    float co = 0.9f;
    velWorld.x = co * (velWorld.x + HPF_velx.input((LPF_accx.input((float)aaWorld.x)) / 4096.0f * dt));
    velWorld.y = co * (velWorld.y + HPF_vely.input((LPF_accy.input((float)aaWorld.y)) / 4096.0f * dt));
    velWorld.z = co * (velWorld.z + HPF_velz.input((LPF_accz.input((float)aaWorld.z)) / 4096.0f * dt));
    */
    
    movWorld.x = HPF_movx.input(velWorld.x * dt);
    movWorld.y = HPF_movy.input(velWorld.y * dt);
    movWorld.z = HPF_movz.input(velWorld.z * dt);

    if(DebugMonitor) {
      delay(30);
      /*Serial.print("dt,dt2:"); Serial.print("\t");
      Serial.print(dt,4); Serial.print("\t");
      Serial.print(dt*dt,4); Serial.print("\t");*/
      Serial.print(oldt); Serial.print("\t");
      Serial.print("vel:"); Serial.print("\t");
      Serial.print(velWorld.x,8); Serial.print("\t");
      Serial.print(velWorld.y); Serial.print("\t");
      Serial.print(velWorld.z); Serial.print("\t");
      Serial.print("mov:"); Serial.print("\t");
      Serial.print(movWorld.x,8); Serial.print("\t");
      Serial.print(movWorld.y); Serial.print("\t");
      Serial.print(movWorld.z); Serial.print("\t");
      /*Serial.print("hpvel:"); Serial.print("\t");
      Serial.print(hpvelx, 4); Serial.print("\t");
      Serial.print(hpvely, 4); Serial.print("\t");
      Serial.print(hpvelz, 4); Serial.print("\t");
      Serial.print("hpmov:"); Serial.print("\t");
      Serial.print(hpmovx, 4); Serial.print("\t");
      Serial.print(hpmovy, 4); Serial.print("\t");
      Serial.print(hpmovz, 4); Serial.print("\t");*/
    }
    
    #ifdef OUTPUT_READABLE_QUATERNION
      if(DebugMonitor){
        // display quaternion values in easy matrix form: w x y z
        Serial.print("quat\t");
        Serial.print(q.w);
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.print(q.z);
        Serial.print("\t");
      }
    #endif

    #ifdef OUTPUT_READABLE_EULER
      if(DebugMonitor){
        // display Euler angles in degrees
        mpu.dmpGetEuler(euler, &q);
        Serial.print("euler\t");
        Serial.print(euler[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(euler[1] * 180/M_PI);
        Serial.print("\t");
        Serial.print(euler[2] * 180/M_PI);
        Serial.print("\t");
      }
    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      if(DebugMonitor){
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // display Euler angles in degrees
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[2] * 180/M_PI);
        Serial.print("\t");
      }
    #endif

    #ifdef OUTPUT_READABLE_REALACCEL
      if(DebugMonitor){
        // display real acceleration, adjusted to remove gravity
        Serial.print("areal\t");
        Serial.print(aaReal.x);
        Serial.print("\t");
        Serial.print(aaReal.y);
        Serial.print("\t");
        Serial.print(aaReal.z);
        Serial.print("\t");
      }
    #endif

    #ifdef OUTPUT_READABLE_WORLDACCEL
      if(DebugMonitor){
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        Serial.print("aworld\t");
        Serial.print(aaWorld.x);
        Serial.print("\t");
        Serial.print(aaWorld.y);
        Serial.print("\t");
        Serial.print(aaWorld.z);
        Serial.print("\t");
      }
    #endif

    if(DebugMonitor) Serial.print("\n");

    if(!DebugMonitor){
      if (Serial.available()) {
        if (Serial.read() == 's') {
          Serial.write(0xff) ;
          int precision = 10000;
          sendFloatin16Bit(q.w, precision);
          sendFloatin16Bit(q.x, precision);
          sendFloatin16Bit(q.y, precision);
          sendFloatin16Bit(q.z, precision);
          sendFloatin16Bit(movWorld.x, precision);
          sendFloatin16Bit(movWorld.y, precision);
          sendFloatin16Bit(movWorld.z, precision);
        }
      }
    }
    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

/* functions to transfer float through serial, to be used by unity */
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

void sendIntin16Bit(int16_t arg){
  uint8_t sign = 3;
  if(arg < 0) {
    sign -= 2;
    arg *= -1; 
  }
  uint8_t first_half = arg>>8;
  uint8_t second_half = arg & MASK_8bit;

  Serial.write(sign);
  Serial.write(first_half);
  Serial.write(second_half);
  Serial.flush();
}
