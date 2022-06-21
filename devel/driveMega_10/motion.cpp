#include "motion.h"


void Motion::begin() {
  if (stepperRight) {
    stepperRight->setDirectionPin(PIN_STEPPER_RIGHT_DIR,false);
    stepperRight->setEnablePin(PIN_STEPPER_RIGHT_ENABLE);
    stepperRight->setAutoEnable(false);
    stepperRight->setSpeedInHz(STEPPER_DEFAULT_SPEED);
    stepperRight->setAcceleration(STEPPER_DEFAULT_ACCEL);
    stepperRight->enableOutputs();
    Serial.println("STEPPER R - INIT OK");
  }
  
  if (stepperLeft) {
    stepperLeft->setDirectionPin(PIN_STEPPER_LEFT_DIR);
    stepperLeft->setEnablePin(PIN_STEPPER_LEFT_ENABLE);
    stepperLeft->setAutoEnable(false);
    stepperLeft->setSpeedInHz(STEPPER_DEFAULT_SPEED);  
    stepperLeft->setAcceleration(STEPPER_DEFAULT_ACCEL);
    stepperLeft->enableOutputs();
    Serial.println("STEPPER L - INIT OK");
  }
}


void Motion::disableMotors(bool value) {
  if (value==0) {
    stepperLeft->enableOutputs();
    stepperRight->enableOutputs();
  }
  else {
    stepperLeft->disableOutputs();
    stepperRight->disableOutputs();
  }
   
}
void Motion::stop() {
  if (status==MOTION_EMERGENCY) return;
  status=MOTION_STOPPED;
  stepperRight->stopMove();
  stepperLeft->stopMove();
  Serial.println("STOP");
}
void Motion::emergencyStop() {
  if (emergencyOverride==false) {
    if (status != MOTION_EMERGENCY) 
      Serial.println("ESTOP");
    status=MOTION_EMERGENCY;
    stepperRight->stopMove();
    stepperLeft->stopMove();
    
  }
}

void Motion::pause(){
  if (status==MOTION_EMERGENCY) return;
  pauseTimer=millis();
  if (status != MOTION_PAUSED && verbose)
    Serial.println("PAUSE");
  status=MOTION_PAUSED;

  stepperRight->stopMove();
  stepperLeft->stopMove();
  
}
void Motion::resume(){
  if (status==MOTION_EMERGENCY) return;
  if (status==MOTION_PAUSED && millis()-pauseTimer > MOTION_RESUME_DELAY) {
    status=MOTION_STOPPED;
    if (verbose)
      Serial.println("RESUME");
  }
  else
    status=MOTION_RESUMING;
}

void Motion::fastForward() {
  if (status==MOTION_EMERGENCY) return;
  status=MOTION_RUNNING;
  stepperRight->setAcceleration(accel);
  stepperLeft->setAcceleration(accel);
  stepperRight->setSpeedInHz(speed);
  stepperLeft->setSpeedInHz(speed);
  stepperRight->runForward();
  stepperLeft->runForward();
  if (verbose){
    Serial.print("Moving FORWARD at:");
    Serial.println(speed);
  }
}

void Motion::forward() {
  if (status==MOTION_EMERGENCY) return;
  status=MOTION_RUNNING;
  stepperRight->setAcceleration(accel);
  stepperLeft->setAcceleration(accel);
  stepperRight->setSpeedInHz(speed/2);
  stepperLeft->setSpeedInHz(speed/2);
  stepperRight->runForward();
  stepperLeft->runForward();
  if (verbose){
    Serial.print("Moving FORWARD at:");
    Serial.println(speed);
  }
}

void Motion::backwards() {
  if (status==MOTION_EMERGENCY) return;
  status=MOTION_RUNNING;
  stepperRight->setAcceleration(accel);
  stepperLeft->setAcceleration(accel);
  stepperRight->setSpeedInHz(speed/2);
  stepperLeft->setSpeedInHz(speed/2);
  stepperRight->runBackward();
  stepperLeft->runBackward();
  if (verbose){
    Serial.print("Moving BACKWARDS at:");
    Serial.println(speed);
  }
}

void Motion::turnRight90() {
  if (status==MOTION_EMERGENCY) return;
  status=MOTION_RUNNING;
  stepperRight->setAcceleration(accel/2);
  stepperRight->setSpeedInHz(300);
  stepperLeft->setAcceleration(accel/2);
  stepperLeft->setSpeedInHz(turnSpeed);
  stepperRight->runForward();
  stepperLeft->runForward();
}
void Motion::turnLeft90() {
  if (status==MOTION_EMERGENCY) return;
  status=MOTION_RUNNING;
  stepperLeft->setAcceleration(accel/2);
  stepperLeft->setSpeedInHz(300);
  stepperRight->setAcceleration(accel/2);
  stepperRight->setSpeedInHz(turnSpeed);
  stepperRight->runForward();
  stepperLeft->runForward();
  
}


void Motion::turnRight90_inPlace() {
  if (status==MOTION_EMERGENCY) return;
  status=MOTION_RUNNING;
  stepperLeft->setAcceleration(accel/2);
  stepperLeft->setSpeedInHz(turnSpeed);
  stepperRight->stopMove();
  stepperLeft->runForward();
}
void Motion::turnLeft90_inPlace() {
  if (status==MOTION_EMERGENCY) return;
  status=MOTION_RUNNING;
  stepperRight->setAcceleration(accel/2);
  stepperRight->setSpeedInHz(turnSpeed);
  stepperRight->runForward();
  stepperLeft->stopMove();
  
}


void Motion::aTurn (int dir) {
  if (status==MOTION_EMERGENCY) return;
  status=MOTION_RUNNING;
  if (dir==MOTION_TURN_LEFT) {
    stepperRight->setSpeedInHz(speed*1.1);
    stepperLeft->setSpeedInHz(speed*0.95);
  }
  else if (dir==MOTION_TURN_RIGHT) {
    stepperRight->setSpeedInHz(speed*0.95);
    stepperLeft->setSpeedInHz(speed*1.1);
  }
  stepperRight->runForward();
  stepperLeft->runForward();
}

void Motion::bTurn (int dir) {
  if (status==MOTION_EMERGENCY) return;
  status=MOTION_RUNNING;
  if (dir==MOTION_TURN_LEFT) {
    stepperRight->setSpeedInHz(speed*1.0);
    stepperLeft->setSpeedInHz(speed*0.85);
  }
  else if (dir==MOTION_TURN_RIGHT) {
    stepperRight->setSpeedInHz(speed*0.85);
    stepperLeft->setSpeedInHz(speed*1.0);
  }
  stepperRight->runForward();
  stepperLeft->runForward();
}

void Motion::cTurn (int dir) {
  if (status==MOTION_EMERGENCY) return;
  status=MOTION_RUNNING;
  if (dir==MOTION_TURN_LEFT) {
    stepperRight->setSpeedInHz(speed*1.05);
    stepperLeft->setSpeedInHz(speed*0.7);
  }
  else if (dir==MOTION_TURN_RIGHT) {
    stepperRight->setSpeedInHz(speed*0.7);
    stepperLeft->setSpeedInHz(speed*1.05);
  }
  stepperRight->runForward();
  stepperLeft->runForward();
}

void Motion::runControl(float speedRight, float speedLeft) {
  if (status==MOTION_EMERGENCY) 
    return;
  lastSpeedUpdate=millis();
  status=MOTION_RUNNING;
  stepperRight->setAcceleration(accel);
  stepperLeft->setAcceleration(accel);

  //#define STEPPER_STEPS_PER_TURN 800
  //#define MOTION_STEPPER_RATIO  3.0
  //#define MOTION_WHEEL_PERIMETER 0.31415
  //speedMS = ((speedHz / STEPPER_STEPS_PER_TURN) / MOTION_STEPPER_RATIO) * MOTION_WHEEL_PERIMETER
  //speedHZ = (speedMS/MOTION_WHEEL_PERIMETER) * MOTION_STEPPER_RATIO * STEPPER_STEPS_PER_TURN
  
  long sr=long ((speedRight/MOTION_WHEEL_PERIMETER) * MOTION_STEPPER_RATIO * STEPPER_STEPS_PER_TURN);
  long sl=long ((speedLeft/MOTION_WHEEL_PERIMETER) * MOTION_STEPPER_RATIO * STEPPER_STEPS_PER_TURN);

  if (abs(sr) > MOTION_SETSPEED_MAX || abs(sl) > MOTION_SETSPEED_MAX) {
    sr=0;
    sl=0;
  }
  
  stepperRight->setSpeedInHz(abs(sr));
  stepperLeft->setSpeedInHz(abs(sl));
  if (speedRight>0)
    stepperRight->runForward();
  else if (speedRight<0)
    stepperRight->runBackward();
  else
    stepperRight->stopMove();
  
  if (speedLeft>0)
    stepperLeft->runForward();
  else if (speedLeft<0)
    stepperLeft->runBackward();
  else
    stepperLeft->stopMove();
    
  if (verbose){
    Serial.print("Moving Controlled at (R,L):");
    Serial.print(speedRight);
    Serial.print(" / ");
    Serial.print(sr);
    Serial.print(" , ");
    Serial.print(speedLeft);
    Serial.print(" / ");
    Serial.println(sl);
  }
}
void Motion::checkSpeedUpdateTimeout() {
  if (millis()-lastSpeedUpdate > MOTION_SPEED_UPDATE_TIMEOUT && status==MOTION_RUNNING) {
    Serial.println ("Speed update TIMEOUT");
    stop();
  }  
}

int32_t Motion::getMotorPosition(int motor) {
  if (motor==STEPPER_LEFT)
    return stepperLeft->getCurrentPosition();
  else if (motor==STEPPER_RIGHT)
    return stepperRight->getCurrentPosition();
  return 0;
}
float Motion::getMotorPositionMts(int motor) {
  //return (float(getStepperSpeed(stepper)) / STEPPER_STEPS_PER_TURN) / MOTION_STEPPER_RATIO * MOTION_WHEEL_PERIMETER;
  return (float(getMotorPosition(motor)) / STEPPER_STEPS_PER_TURN) / MOTION_STEPPER_RATIO * MOTION_WHEEL_PERIMETER;

}

int Motion::setSpeed(int sp){
  if (sp>MOTION_SETSPEED_MAX)
    sp=MOTION_SETSPEED_MAX;
  if (sp<MOTION_SETSPEED_MIN)
    sp=MOTION_SETSPEED_MIN;
    
  float multiplier=float(sp) / float(speed);
  speed=sp;
  turnSpeed=speed/2;
  if (verbose) {
    Serial.print ("NEW SPEED: ");
    Serial.print (sp);
    Serial.print (" RATIO (NEW / OLD): ");
    Serial.println (multiplier);
  }
  
  //REVISAR Y PROBAR - AJUSTA LA VELOCIDAD DEL MOVIMIENTO ACTUAL
  stepperRight->setSpeedInMilliHz(stepperRight->getSpeedInMilliHz() * multiplier);
  stepperLeft->setSpeedInMilliHz(stepperLeft->getSpeedInMilliHz() * multiplier);
  stepperRight->applySpeedAcceleration();
  stepperLeft->applySpeedAcceleration();
}

int Motion::setAccel(int ac){
  accel=ac;
  stepperRight->applySpeedAcceleration();
  stepperLeft->applySpeedAcceleration();
}

float Motion::getStepperSpeedMs (bool stepper) {
  return (float(getStepperSpeed(stepper)) / STEPPER_STEPS_PER_TURN) / MOTION_STEPPER_RATIO * MOTION_WHEEL_PERIMETER;
}

int Motion::getStepperSpeed (bool stepper) {
  if (stepper == STEPPER_RIGHT) 
    return (stepperRight->getCurrentSpeedInMilliHz() / 1000);
  else
    return (stepperLeft->getCurrentSpeedInMilliHz() / 1000);
}

bool Motion::motionEnabled() {
  if (status==MOTION_EMERGENCY) 
    return false;
  else if (status==MOTION_PAUSED) 
    return false;
  return true;  
}
