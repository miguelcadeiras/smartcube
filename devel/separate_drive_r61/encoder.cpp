#include "encoder.h"
#include "arduino.h"
void Encoder::begin() {
  pinMode(PIN_ENCODER_A, INPUT_PULLUP); // internal pullup input pin 2 
  pinMode(PIN_ENCODER_B, INPUT_PULLUP); // internal pullup input pin 3

}
void Encoder::update() {
  if (counter != counterPrev) {
    if (counter < counterPrev)
      counter=0;
    counterPrev=counter;
    
    if (counter > ENCODER_STEPS_PER_5CM) {
 
      lastIncrementTime=millis();
      counter=0;
      distanceCm+=5;
      distanceMts=distanceCm/100;
      //MOSTRAR POR SERIAL
      Serial.print("{mts;");
      Serial.print(distanceMts);
      Serial.println(";}");
      //Serial.println (distanceMts);
    }
  }
}
void Encoder::resetDistance() {
  counter=0;
  distanceCm=0;
  distanceMts=0;
  lastIncrementTime=millis();
}
void Encoder::isr0 () {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(PIN_ENCODER_B)==LOW) {
    //counter--;
  } else{
    counter++;
  }
}


void Encoder::isr1 () {
  // ai1 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(PIN_ENCODER_A)==LOW) {
    counter++;
  } else{
    counter--;
  }
}
