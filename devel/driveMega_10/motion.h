#ifndef MOTION_H
#define MOTION_H
#include <FastAccelStepper.h> 
#include "config.h"

#define MOTION_STOPPED     0
#define MOTION_RUNNING     1
#define MOTION_EMERGENCY   2
#define MOTION_PAUSED      3
#define MOTION_RESUMING    4

#define STEPPER_RIGHT      0
#define STEPPER_LEFT       1


class Motion {
    int accel=STEPPER_DEFAULT_ACCEL;
    int speed=STEPPER_DEFAULT_SPEED;
    int turnSpeed=STEPPER_DEFAULT_SPEED/2;
    int status=MOTION_STOPPED;
    unsigned long lastSpeedUpdate=0;
    
    bool emergencyOverride=false;
    unsigned long pauseTimer=0;
    
    
  public:
    FastAccelStepper *stepperRight = NULL;
    FastAccelStepper *stepperLeft = NULL;
    bool verbose=false;
    
    void begin();
    int getStatus() {return status;};
    void disableMotors(bool value);
    void stop();
    void pause();
    void resume();
    void emergencyStop();
    void fastForward();
    void forward();
    void backwards();
    void turnRight90();
    void turnLeft90();
    void turnRight90_inPlace();
    void turnLeft90_inPlace();
    void aTurn(int dir);
    void bTurn(int dir);
    void cTurn(int dir);
    int32_t getMotorPosition(int motor);
    float getMotorPositionMts(int motor);
    
    void runControl (float speedRight, float speedLeft);
    void checkSpeedUpdateTimeout();

    void resetEmergency() { status=MOTION_STOPPED; }

    int getAccel() { return accel; }
    int getSpeed() { return speed; }
    int setSpeed(int sp);
    int setAccel(int ac);
    int getStepperSpeed (bool stepper);
    float getStepperSpeedMs (bool stepper);
    bool motionEnabled();
 
};




#endif
