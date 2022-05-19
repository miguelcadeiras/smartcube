#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include "arduino.h"

#define US_IDLE                 0
#define US_WAITING_ECHO_HIGH    1
#define US_WAITING_ECHO_LOW     2
#define US_READY                3

#define US_TIMEOUT              80      // Milliseconds
#define US_INTERVAL             150     // Milliseconds
#define US_MAX_DISTANCE         250     // Centimetros
#define US_ERROR_DISTANCE       600
class UltrasonicSensor {
  private:
    int pinTrigger = 0;
    int pinEcho = 0;
    int distance = -1;
    int status = US_IDLE;
    bool useInterrupt=true;
    unsigned long lastTrigger = 0;
    unsigned long pulseStartTime = 0;
    unsigned long pulseEndTime = 0;
    void calcDistance();
    

  public:

  
    UltrasonicSensor(int pTrigger, int pEcho, bool useInt=true);
    bool trigger();
    void update();
    void isr();
    int getPinEcho() {return pinEcho;};
    int getDistance() { return distance;};
};


#endif
