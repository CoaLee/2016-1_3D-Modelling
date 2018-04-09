#define DebugMonitor true //If you want to check values through serial monitor, set it true.

#include <Filters.h>
#include "I2Cdev.h"
#include "MPU9150_9Axis_MotionApps41.h"
//#include "MPU9150.h" // not necessary if using MotionApps include file

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU9150 mpu;

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
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
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
    //TOCHECK if(DebugMonitor) Serial.println(F("FIFO overflow!"));
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
    
    movWorld.x = HPF_movx.input(velWorld.x * dt);
    movWorld.y = HPF_movy.input(velWorld.y * dt);
    movWorld.z = HPF_movz.input(velWorld.z * dt);

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
