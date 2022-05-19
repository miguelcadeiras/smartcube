#include <FastAccelStepper.h>   //https://github.com/gin66/FastAccelStepper

#include "config.h"
#include "motion.h"
#include "ultrasonic.h"
#include "reflective.h"
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


//Sensores ultrasonido
UltrasonicSensor usSensor1 (PIN_US1_TRIGGER, PIN_US1_ECHO);
UltrasonicSensor usSensor2 (PIN_US2_TRIGGER, PIN_US2_ECHO);
UltrasonicSensor *usSensors[US_SENSORS_COUNT]={&usSensor1, &usSensor2};
uint8_t usSensorsIndex = 0;
unsigned long usSensorsUpdateTimer = 0;

//Sensor de piso e inclinacion
ReflectiveSensor refSensor (PIN_REFLECTIVE_SENSOR);
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

  //attachInterrupt (digitalPinToInterrupt(usSensor1.getPinEcho()), isrWrapperUs1 , CHANGE);
  //attachInterrupt (digitalPinToInterrupt(usSensor2.getPinEcho()), isrWrapperUs2 , CHANGE);

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
  
  if (motion.getStatus() == MOTION_RUNNING)
  { } 
  else 
    encoder.resetTimer();
    
  // SI ESTA ANDANDO EN PIXY DRIVE Y NO INCREMENTE EL ENCODER POR MAS DE CIERTO TIEMPO PARA. 
  if (driveMode==DRIVE_AUTO && motion.getStatus() == MOTION_RUNNING && encoder.getLastIncrementTime() > ENCODER_NOT_MOVING_TIMEOUT && noEncoderDetection) {
    buzzer.beep(300);
    serialMsg (MSG_EX, MSG_EX_ENOREADING);
    driveMode=DRIVE_MANUAL;
    motion.stop();
  }

 // SI ESTA EN MODO AUTO Y NO ESTA PAUSADO (POR ULTRASONIDO) CONTROLA EL PIXY
 // SI NO VACIA EL BUFFER SERIAL
 if (driveMode==DRIVE_AUTO /*&& motion.getStatus() != MOTION_PAUSED*/) {
      pixyDriveUpdate();

  } else {

    pixyDriveFlushSerial();
  }

  // RUTINA DE DISPARO DE LOS ULTRASONIDO Y ACTUALIZACION
  if (millis()-usSensorsUpdateTimer > US_INTERVAL) {
    // PARA INTERCALAR EL TRIGGER DE LOS US: 
    usSensors[usSensorsIndex]->trigger();
    // PARA DISPARARLOS SIMULTANEAMENTE
    //usSensors[0]->trigger();
    //usSensors[1]->trigger();
    
    usSensorsIndex++;
    if (usSensorsIndex >= US_SENSORS_COUNT)
      usSensorsIndex=0;
    usSensorsUpdateTimer = millis();
  }
  usSensor1.update();
  usSensor2.update();


  tiltSensor.update();
  
  vccMotor.update();
  vccComp.update();
  
  //DEBUG DE LOS SENSORES ULTRASONICOS Y ACELEROMETRO
  static unsigned long t1=0;
  if (millis() - t1 > 250 && verbose) {
    t1=millis();
    Serial.print ("US1: ");
    Serial.print (usSensor1.getDistance());
    Serial.print (" - US2: ");
    Serial.print (usSensor2.getDistance());

    
    if (tiltSensor.available()) {
      Serial.print (" - ACCEL (X,Y,Z): ");
      Serial.print (tiltSensor.getX());
      Serial.print (",");
      Serial.print (tiltSensor.getY());
      Serial.print (",");
      Serial.print (tiltSensor.getZFloat(), 1);
    }
    
    Serial.println();
 
  }


  
 

 

  // SI ESTA EN MODO AUTO Y HAY UN OBJETO CERCA, PAUSA LA MARCHA INDEFINIDAMENTE
  //if (driveMode==DRIVE_AUTO && motion.getStatus() != MOTION_STOPPED && ultrasoundDetection) {
  if (motion.getStatus() != MOTION_STOPPED && ultrasoundDetection) {
    bool pause=false;
    // BAJA LA VELOCIDAD AL ACERCARSE A UN OBSTACULO - TODAVIA NO IMPLEMENTADO - A PROBAR
 /*   if (usSensor1.getDistance() >US_MIN_DISTANCE && usSensor1.getDistance() <US_MIN_DISTANCE+30) {
      motion.setSpeed(STEPPER_DEFAULT_SPEED/2);
    }
    if (usSensor1.getDistance() >US_MIN_DISTANCE+60) {
      motion.setSpeed(STEPPER_DEFAULT_SPEED);
    }
*/
    if (lidarPause==true)
      pause=true;
/*    if (usSensor1.getDistance() >=0 && usSensor1.getDistance() <=US_MIN_DISTANCE)
      pause=true;
    if (usSensor2.getDistance() >=0 && usSensor2.getDistance() <=US_MIN_DISTANCE)
      pause=true;*/

    
    if (pause==true && motion.getStatus() != MOTION_PAUSED) {
      serialMsg (MSG_EX, MSG_EX_UPAUSE);
      motion.pause();
    }
    else if (pause==false && motion.getStatus() == MOTION_PAUSED) {
      serialMsg (MSG_EX, MSG_EX_URESUME);
      motion.resume();
    }
    else if (pause==false && motion.getStatus() == MOTION_RESUMING) {
      motion.resume();
    }
  }

  // SENSADO DE TILT

  if (motion.getStatus() != MOTION_STOPPED && tiltDetection && motion.getStatus() != MOTION_EMERGENCY )
    if (abs(tiltSensor.getX()) >= TILT_MAX_ANGLE_X || abs(tiltSensor.getY()) >= TILT_MAX_ANGLE_Y) {
      serialMsg (MSG_EX, MSG_EX_CRITICTILT);
      buzzer.beep(500);
      motion.emergencyStop();
    }



}
//-------------------------------------------------------------------------------------------------------------------

// WRAPPERS DE LAS RUTINAS ISR PARA USAR DESDE UNA CLASE
void isrWrapperEnc0() {
  encoder.isr0();
}
void isrWrapperUs1() {
  usSensor1.isr();
}
void isrWrapperUs2() {
  usSensor2.isr();
}

// SETEO DE INTERRUPCIONES DE LOS ULTRASONIDOS
void setupUSInterrupts() {
  // Habilita PCINT para PCINT23:16
  bitSet (PCICR, PCIE2);
  // Habilita la Interrupcion para 
  bitSet (PCMSK2, PIN_US1_ECHO_INT );
  bitSet (PCMSK2, PIN_US2_ECHO_INT );
}
ISR(PCINT2_vect) {
  isrWrapperUs1();
  isrWrapperUs2();
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
