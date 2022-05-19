#include "Arduino.h"
#include "ultrasonic.h"
#include "config.h"


UltrasonicSensor::UltrasonicSensor(int pTrigger, int pEcho, bool useInt=true) {
  pinTrigger=pTrigger;
  pinEcho=pEcho;
  useInterrupt=useInt;
  pinMode (pinTrigger, OUTPUT);
  digitalWrite (pinTrigger, LOW);
  pinMode (pinEcho, INPUT_PULLUP);

}

bool UltrasonicSensor::trigger() {
  if (status!=US_IDLE)
    return false;
  
  digitalWrite (pinTrigger, HIGH);
  delayMicroseconds(10);
  digitalWrite (pinTrigger, LOW);
  lastTrigger=millis();
  status=US_WAITING_ECHO_HIGH;
  return true;
}

void UltrasonicSensor::isr() {
  uint8_t val=digitalRead(pinEcho);
  if (val==HIGH && status==US_WAITING_ECHO_HIGH) {
    pulseStartTime=micros();
    status=US_WAITING_ECHO_LOW;
  }
  if (val==LOW && status==US_WAITING_ECHO_LOW) {
    pulseEndTime=micros();
    status=US_READY;
  }
}
void UltrasonicSensor::update() {

  if (useInterrupt==false)
    isr();
  
  if (status==US_WAITING_ECHO_HIGH && millis()-lastTrigger > US_TIMEOUT) {
    distance=-1;
    status=US_IDLE;
  }
  else if (status==US_WAITING_ECHO_LOW && millis()-lastTrigger > US_TIMEOUT) {
    distance=-1;
    status=US_IDLE;
  }
  
  /*if (status==US_IDLE && millis()-lastTrigger > US_INTERVAL) {
    trigger();
  }*/
  if (status==US_READY) {
    calcDistance();
    status=US_IDLE;
  }

}
void UltrasonicSensor::calcDistance() {
  unsigned long d=(pulseEndTime-pulseStartTime) / 58;
  if (d <= US_MAX_DISTANCE)
    distance=d;
  if (d > US_MAX_DISTANCE && d < US_MAX_DISTANCE*3)
    distance=-1;
}
