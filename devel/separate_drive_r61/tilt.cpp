//#include "tilt.h"
/*
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
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    angZ=euler[0] * 180/M_PI;
    angX=euler[1] * 180/M_PI;
    angY=euler[2] * 180/M_PI;
    newData=true;
  }
}
bool TiltSensor::available() {
  if (newData==true) {
    newData=false;
    return true;
  }
  return false;
}*/
