#ifndef TILT_H
#define TILT_H

#include "config.h"
#include <arduino.h>
//#include <MPU6050_6Axis_MotionApps20.h>
#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include <I2Cdev.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

class TiltSensor {
  private:
    MPU6050 mpu;
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    float angX=0;
    float angY=0;
    float angZ=0;
    bool newData=false;
    unsigned long timer=0;
    
  public:
    void begin();
    void update();
    bool available();
    int getX() {return angX;};
    int getY() {return angY;};
    int getZ() {return angZ;};
    float getZFloat() {return angZ;};

    
};

void TiltSensor::begin() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    Serial.print ("MPU Calibration: ");
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    
    
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();
    
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
}
 

void TiltSensor::update() {
  if (!dmpReady)
    return;
  if (millis()-timer < TILT_ACCELEROMETER_UPDATE_INTERVAL)
    return;
  timer=millis();
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    
   /* mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    angZ=euler[0] * 180/M_PI;
    angX=euler[1] * 180/M_PI;
    angY=euler[2] * 180/M_PI;
   */
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    angZ=ypr[0] * 180/M_PI;
    angX=ypr[1] * 180/M_PI;
    angY=ypr[2] * 180/M_PI;
   
    
    
    newData=true;
  }
}
bool TiltSensor::available() {
  if (newData==true) {
    newData=false;
    return true;
  }
  return false;
}

#endif
