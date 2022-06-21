#ifndef ENCODER_H
#define ENCODER_H
#include "config.h"
#include "arduino.h"

class Encoder {
  private:
    volatile int counter;
    int counterPrev;
    unsigned long lastIncrementTime=0;
    float distanceCm=0;
    float distanceMts=0;
    float speed=0;
    
  public:
    void isr0();
    void isr1();
    void begin();
    void available();
    void update();
    void resetDistance();
    void resetTimer() {lastIncrementTime=millis();}
    float getCm() {return distanceCm;};
    float getMts() {return distanceMts;};
    unsigned long getLastIncrementTime() {return millis() - lastIncrementTime;};
};


#endif
