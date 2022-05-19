#ifndef EXTRAS_H
#define EXTRAS_H
#include "config.h"
#include "arduino.h"

#define LED_OFF     0
#define LED_ON      1
#define LED_BLINK   2

#define VOLTMEASURE_IN_OFFSET   0.45          // CAIDA DE TENSION EN LOS DIODOS DE ENTRADA

class StatusLed {
    unsigned long lastChange=0;
    uint8_t status=LED_OFF;
    int blinkInterval=250;
    uint8_t pinLed;
  public:
    StatusLed(uint8_t pLed);
    void update();
    void setOn();
    void setOff();
    void setBlink();
    void setBlink(int interval) {
       blinkInterval=interval;
       setBlink();
    }
};

class Buzzer {
    uint8_t pinBuzzer;
    unsigned long timer;
    bool status=LOW;
  public:
    void update();
    void beep (int t);
    void off ();
    void on ();
    Buzzer (uint8_t pBuzzer);
    
};

class VoltMeasure {
    uint8_t pinMeasure;
    float vcc=4.95;
    long res1;
    long res2;
    bool groundOffset;
    float voltage=0.0;
    unsigned long timer;
  public:
    VoltMeasure(uint8_t pin, long r1, long r2, bool gndOffset=false);
    float read();
    void update();
};


#endif
