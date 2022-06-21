#include "extras.h"
/*---------------------------------------------------------------------------------------------------------------------------------*/
Buzzer::Buzzer (uint8_t pBuzzer) {
  pinBuzzer = pBuzzer;
  pinMode (pinBuzzer, OUTPUT);
  off();
}

void Buzzer::update() {
  if (status==HIGH && millis() >= timer)
    off();
  else if (status==LOW && millis() >= timer && repeticiones > 0)
    on();

}

void Buzzer::beep (int t, uint8_t rep=1) {
  if (rep > 10)
    rep=10;
  if (t > 3000)
    t=3000;
  beepLen=t;
  repeticiones=rep;
  
  on();
}

void Buzzer::on() {
  if (repeticiones > 0) {
    repeticiones--;
    status=HIGH;
    timer=millis() + beepLen;
    digitalWrite (pinBuzzer, HIGH);
  }
}

void Buzzer::off() {
  status=LOW;
  timer=millis() + beepLen;
  digitalWrite (pinBuzzer, LOW);  
}
/*---------------------------------------------------------------------------------------------------------------------------------*/
StatusLed::StatusLed(uint8_t pLed) {
  pinLed = pLed;
  pinMode (pinLed, OUTPUT);
}
void StatusLed::update() {
  
  if (status==LED_BLINK) {
    if (millis() - lastChange > blinkInterval) {
      lastChange=millis();
      bool s=not(digitalRead(pinLed));
      digitalWrite (pinLed, s);
    }
  }
}

void StatusLed::setBlink() {
  status=LED_BLINK;
  digitalWrite (pinLed, HIGH);
}

void StatusLed::setOn() {
  status=LED_ON;
  digitalWrite (pinLed, HIGH);
}
void StatusLed::setOff() {
  status=LED_OFF;
  digitalWrite (pinLed, LOW);
}
/*---------------------------------------------------------------------------------------------------------------------------------*/
VoltMeasure::VoltMeasure(uint8_t pin, long r1, long r2, bool gndOffset=false) {
  pinMeasure = pin;
  pinMode (pinMeasure, INPUT);
  res1=r1;
  res2=r2;
  groundOffset=gndOffset;
}
void VoltMeasure::update() {
  if (millis() - timer < 50)
    return;
  timer=millis();
  
  int adcVal=0;
  float vIn=0.0;
  for (int n=0; n < 5; n++) 
    adcVal+=analogRead(pinMeasure);
  adcVal=adcVal/5;

  vIn= ((vcc/1023) * adcVal) * (res1+res2) / res2;

  if (vIn <= 0.5) {
    voltage=0.0;
    return;
  }
  
  //CAIDA DE TENSION DEL DIODIO EN V+
  vIn+=VOLTMEASURE_IN_OFFSET;
  //CAIDA DE TENSION DEL DIODIO EN GND
  if (groundOffset==true)
    vIn+=VOLTMEASURE_IN_OFFSET;

  // adc_Eavg = α * adc_raw + ( 1 - α ) * adc_Eavg ;
  voltage = 0.25f * vIn + (1-0.25f) * voltage;
  //return vIn;
}
float VoltMeasure::read() {
  return voltage;
}
void VoltMeasure::readStr(char *buf) {
  dtostrf (voltage,4,1,buf);
}

/*---------------------------------------------------------------------------------------------------------------------------------*/
