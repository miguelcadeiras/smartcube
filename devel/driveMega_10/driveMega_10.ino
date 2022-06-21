#include <FastAccelStepper.h>   //https://github.com/gin66/FastAccelStepper

#include "config.h"
#include "motion.h"
#include "tilt.h"
#include "encoder.h"
#include "messages.h"
#include "extras.h"


// pyxy_dt_r4
String ver=VERSION;

//vervose mode imprime todas las informaciones en el serial
bool verbose = false;

int driveMode = DRIVE_MANUAL;
unsigned long pixyDriveTimer = 0;

// steppers y Motion
FastAccelStepperEngine stepperEngine = FastAccelStepperEngine();
Motion motion;


//Sensor de piso e inclinacion
TiltSensor tiltSensor;
Encoder encoder;

//Varios
StatusLed statusLed(PIN_STATUS_LED);
Buzzer buzzer(PIN_BUZZER);
VoltMeasure vccMotor (PIN_VCC36, 100000, 10000, true);
VoltMeasure vccComp  (PIN_VCC24, 100000, 10000);

// variables para habilitar y deshabilitar funciones de seguridad
bool ultrasoundDetection = true;
bool noEncoderDetection = true;
bool tiltDetection = true;

bool lidarPause=false;

int nextIntersectAction=DRIVE_NEXT_INTERSECT_STRAIGHT;

//-------------------------------------------------------------------------------------------------------------------
void setup()
{ 
  //STARTING SERIAL COM. 
  Serial.begin(115200);
  Serial.println( "separete_drive_5 ");
  
  // Steppers y Motion  
  stepperEngine.init();
  motion.stepperRight=stepperEngine.stepperConnectToPin(PIN_STEPPER_RIGHT_STEP);
  motion.stepperLeft=stepperEngine.stepperConnectToPin(PIN_STEPPER_LEFT_STEP);
  motion.begin();

  
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), isrWrapperEnc0, RISING);
  //attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), isrWrapperEnc1, RISING);
  

  encoder.begin();
  
  pixySetup();

  tiltSensor.begin();

  showInfo();
  statusLed.setBlink (250);

  // CONFIGURA las interrupciones para los sensores de ultrasonido
  setupUSInterrupts();

  Serial1.begin (9600);
}

//-------------------------------------------------------------------------------------------------------------------
void loop() {

  //encoder_loop();

  serialLoop();
  encoder.update();
  buzzer.update();
  statusLed.update();
  tiltSensor.update();
  vccMotor.update();
  vccComp.update(); 
  motion.checkSpeedUpdateTimeout();
  
  /*if (motion.getStatus() == MOTION_RUNNING)
  { } 
  else 
    encoder.resetTimer();
  */
  static unsigned long serialSendTimer=0;
  
  if (millis()-serialSendTimer > 50) { //50
    char buf[200];
    char tBuf[50];

    serialSendTimer=millis();
    
    //TENSIONES
    char bufVmot[10];
    char bufVcomp[10];
    vccMotor.readStr(bufVmot);
    vccComp.readStr(bufVcomp);
    sprintf(buf, "{\"vmot\":%s, \"vcomp\":%s}", bufVmot, bufVcomp);
    Serial.println(buf);

    //ACELEROMETRO
    char bufZ[10], bufY[10], bufX[10];
    dtostrf (tiltSensor.getXFloat(),4,1,bufX);
    dtostrf (tiltSensor.getYFloat(),4,1,bufY);
    dtostrf (tiltSensor.getZFloat(),4,1,bufZ);
    sprintf(buf, "{\"accX\":%s, \"accY\":%s, \"accZ\":%s}", bufX, bufY, bufZ);
    Serial.println(buf);

    //ENCODER
    char bufEnc[10];
    dtostrf (encoder.getMts(),4,3,bufEnc);
    sprintf(buf, "{\"enc\":%s}", bufEnc);
    Serial.println(buf);

    //MOTION
    char bufSpL[10], bufSpR[10], bufDistL[20], bufDistR[20];
    dtostrf (motion.getStepperSpeedMs(STEPPER_LEFT),4,3,bufSpL);
    dtostrf (motion.getStepperSpeedMs(STEPPER_RIGHT),4,3,bufSpR);
    dtostrf (motion.getMotorPositionMts(STEPPER_LEFT),4,3,bufDistL);
    dtostrf (motion.getMotorPositionMts(STEPPER_RIGHT),4,3,bufDistR);
    sprintf(buf, "{\"mstat\":%d, \"spl\":%s, \"spr\":%s, \"dl\":%s, \"dr\":%s}", motion.getStatus(), bufSpL, bufSpR, bufDistL, bufDistR);
    Serial.println(buf);
  }
    
//serialMsg (MSG_VMOT, vccMotor.read());
//serialMsg (MSG_VCOMP, vccComp.read());
  
  //DEBUG DE LOS SENSORES ULTRASONICOS Y ACELEROMETRO
  static unsigned long t1=0;
  if (millis() - t1 > 250 && verbose) {
    t1=millis();
        
    if (tiltSensor.available()) {
      Serial.print (" - ACCEL (X,Y,Z): ");
      Serial.print (tiltSensor.getX());
      Serial.print (tiltSensor.getY());
      Serial.print (",");
      Serial.print (tiltSensor.getZFloat(), 1);
    }
    
    Serial.println();
 
  }


  
 

 

 


}
//-------------------------------------------------------------------------------------------------------------------

// WRAPPERS DE LAS RUTINAS ISR PARA USAR DESDE UNA CLASE
void isrWrapperEnc0() {
  encoder.isr0();
}


// SETEO DE INTERRUPCIONES DE LOS ULTRASONIDOS
void setupUSInterrupts() {
  // Habilita PCINT para PCINT23:16
  bitSet (PCICR, PCIE2);
  // Habilita la Interrupcion para 
  //bitSet (PCMSK2, PIN_US1_ECHO_INT );
  //bitSet (PCMSK2, PIN_US2_ECHO_INT );
}
ISR(PCINT2_vect) {
//  isrWrapperUs1();
//  isrWrapperUs2();
}



// FUNCION PARA ENVIAR MENSAJES VIA SERIAL (EJ: {mts;20.04;});
void serialMsg (char *message, int details) {
  char buf[10];
  sprintf (buf, "%d", details);
  serialMsg (message, buf);
}

void serialMsg (char *message, float details) {
  char buf[10];
  dtostrf (details,4,2,buf);
  serialMsg (message, buf);
}

void serialMsg (char *message, char *details) {
  Serial.print("{");
  Serial.print (message);
  Serial.print(";");
  Serial.print(details);
  Serial.println(";}");
}
