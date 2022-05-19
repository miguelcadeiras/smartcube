#ifndef REFLECTIVE_H
#define REFLECTIVE_H
#include "config.h"
#include <arduino.h>

#define REFLECTIVE_OK     0
#define REFLECTIVE_FAIL   1



class ReflectiveSensor {
    int pinSensor = 0;
    int status=0;
    int sampleIdx=0;
    int samples[REFLECTIVE_AVERAGE_SAMPLES];   
    unsigned long lastReadTime = millis();
  public:
    void update();
    int getStatus();
    ReflectiveSensor(int pSensor) {
      pinSensor=pSensor;
      pinMode (pinSensor, INPUT_PULLUP);
      for (int n=0; n<REFLECTIVE_AVERAGE_SAMPLES; n++)
        samples[n]=0;
    }
};


#endif
