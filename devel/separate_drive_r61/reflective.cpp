#include "reflective.h"

int ReflectiveSensor::getStatus() {
  return status;
}
void ReflectiveSensor::update() {
  if (millis()-lastReadTime > REFLECTIVE_READ_INTERVAL) {
    lastReadTime=millis();
    samples[sampleIdx]=not(digitalRead(pinSensor));
    sampleIdx++;
    if (sampleIdx >= REFLECTIVE_AVERAGE_SAMPLES)
      sampleIdx=0;
   
    status=0;

    for (int n=0; n<REFLECTIVE_AVERAGE_SAMPLES; n++)
      status+=samples[n];
    status=status/REFLECTIVE_AVERAGE_SAMPLES;
    //Serial.print (sampleIdx);
    //Serial.print (" ");
    //Serial.println (status);
  }
}
