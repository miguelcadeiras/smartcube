String ver= "santi";
#include <SoftwareSerial.h>
#include <Pixy2.h>
#include "config.h"
#include "vector.h"

SoftwareSerial SerialS(P_SOFT_RX, P_SOFT_TX);
Pixy2 pixy;
#define defaultPixyCenter  (pixy.frameWidth/2)
#define maxPixyVectorSize pixy.frameHeight

int pixyWidth=0;
int pixyHeight=0;
int vectorsCount=0;
int intersectionCount=0;

PixyVector vect(79,52);

int speedLeft=0;
int speedRight=0;

int32_t  error=0;
int32_t  errorPrev=0;
int32_t  integral;
int32_t  pid;
float pGain=PID_P_GAIN;      // 
float iGain=PID_I_GAIN;      // 
float dGain=PID_D_GAIN;      // 

unsigned long lastVectorTime=0;
unsigned long pixyTimer=0;
bool pixyNoLine=false;
bool lidarStop=false;
unsigned long lidarStopTimer=0;

unsigned int driveMode = DRIVE_MODE_PIXY_ONLY;

void setup(){
    Serial.begin(9600);
    Serial.println ("START!");
    Serial.print ("Pixy INIT: ");
    // Inicializamos Pixy 2
    int i=pixy.init();

    if (i == PIXY_RESULT_OK) {
      pixy.changeProg("line");
      delay(100);
      Serial.print ("PRG OK");
      
      pixy.setLamp(1,0);
      Serial.print ("PRG LAMP");
      
      pixyWidth=pixy.frameWidth;
      pixyHeight=pixy.frameHeight;
    
      Serial.print( " OK (" );
    
      Serial.print (pixyWidth);
      Serial.print ("x");
      Serial.print (pixyHeight);
      Serial.println (")");  
    } else {
      Serial.println ("FAIL");
    }
    SerialS.begin(9600);
    delay(1000);
}

void loop(){

  if (SerialS.available()) {
    char ch=SerialS.read();
    if (ch=='r')
      driveMode=DRIVE_MODE_REALSENSE;
    else if (ch=='p')
      driveMode=DRIVE_MODE_PIXY_ONLY;

   }


   if (millis()-lidarStopTimer > 2000) {
    lidarStopTimer=0;
    lidarStop=false;
   }
   
   realSenseReadSerial();

   

   if (driveMode==DRIVE_MODE_PIXY || driveMode==DRIVE_MODE_PIXY_ONLY) {
    if (millis()-pixyTimer > PIXY_INTERVAL) {
       pixyTimer=millis();
       processPixy();
    }
   }
   

  
   
 /*  if (Serial.available()) {
    char ch=Serial.read();
    if (ch=='r') {
      Serial.println ("RESET INTEGRADOR");
      integral=0;
    }
    else if (ch=='0') {
      pGain=PID_P_GAIN;      
      iGain=PID_I_GAIN;      
      dGain=PID_D_GAIN; 
    }
    else if (ch=='7') 
      pGain+=0.1;
    else if (ch=='4') 
      pGain-=0.1;
    else if (ch=='8') 
      iGain+=0.1;
    else if (ch=='5') 
      iGain-=0.1;
    else if (ch=='9') 
      dGain+=0.1;
    else if (ch=='6') 
      dGain-=0.1;



      
   }
  */
}
