#define PIXY_INPUT_LEVEL  HIGH

#define SERIAL_PIXY_BUFFER_SIZE  30
static uint8_t serialPixyBufferIdx=0;
static unsigned long serialPixyStringTimer=0;
static char serialPixyBuffer[SERIAL_PIXY_BUFFER_SIZE+1];



void pixySetup() {
  pinMode(PIN_PIXY_STRAIGHT, INPUT_PULLUP);
  pinMode(PIN_PIXY_LEFT, INPUT_PULLUP);
  pinMode(PIN_PIXY_RIGHT, INPUT_PULLUP);
  pinMode(PIN_PIXY_LEFT90, INPUT_PULLUP);
  pinMode(PIN_PIXY_RIGHT90, INPUT_PULLUP);
  pinMode(PIN_PIXY_STOP, INPUT_PULLUP);
  pinMode(PIN_PIXY_INTERSECT, INPUT_PULLUP);
}
void pixyDriveUpdate() {
  
  if (Serial1.available()) {
    char ch=Serial1.read();
    // CON ESTA LINEA MUESTRA LO QUE LE LLEGA DEL NANO
    //Serial.write(ch);
    
    if (serialPixyBufferIdx < SERIAL_PIXY_BUFFER_SIZE) {
      serialPixyBuffer[serialPixyBufferIdx]=ch;
      serialPixyBufferIdx++;
      if (ch=='\r') { 
        serialPixyBuffer[serialPixyBufferIdx]='\0';
        serialPixyBufferIdx=0;
        serialPixyParseString(serialPixyBuffer);
      }
    }
    else {
      Serial.println ("STRING OVERFLOW");
      serialPixyBufferIdx=0;
    }
  } 

}
void pixyDriveFlushSerial() {
  if (Serial1.available()) 
    char ch=Serial1.read();
}
void pixyDriveDebugSerial() {
  if (Serial1.available()) 
    Serial.write(Serial1.read());
}


void serialPixyParseString(char *buf) {
  static unsigned long pixyIntersectTimer=0;
  static unsigned long intersectActionTime=PIXY_DRIVE_INTERSECT_TIME_STRAIGHT;

  int sr;
  int sl;
  int cmd;
  int ic=0;
  
  int s=sscanf (buf, "%d;%d;%d;%d", &cmd, &ic, &sr, &sl);

  if (s==4) {
    lidarPause=false;
    if (cmd==0) {
      serialMsg(MSG_EX, MSG_EX_PIXYSTOP);
      driveMode=DRIVE_MANUAL;
      motion.stop();
    }
    else if (cmd==1 || cmd==2) 
      motion.runControl(sr, sl);
    else if (cmd==3) {
      lidarPause=true;
    }

    if (cmd==2) {
      Serial.println("---------  REALSENSE! --------------");
      //Serial.println("---------- REALSENSE! --------------");
    }
    if (ic>0) {
      pixyIntersectTimer=millis();
      serialMsg(MSG_INTER, 1);
      buzzer.beep(100);
    }
  
  }
  
}


void pixyDriveUpdate2(){
  static unsigned long pixyIntersectTimer=0;
  static unsigned long intersectActionTime=PIXY_DRIVE_INTERSECT_TIME_STRAIGHT;

  if (digitalRead(PIN_PIXY_STOP)==PIXY_INPUT_LEVEL) {
    serialMsg(MSG_EX, MSG_EX_PIXYSTOP);
    driveMode=DRIVE_MANUAL;
    motion.stop();
  }
  if (millis()-pixyIntersectTimer < intersectActionTime)
    return;
  if (digitalRead(PIN_PIXY_STRAIGHT)==PIXY_INPUT_LEVEL) motion.fastForward();
  if (digitalRead(PIN_PIXY_LEFT)==PIXY_INPUT_LEVEL) motion.cTurn(MOTION_TURN_LEFT);
  if (digitalRead(PIN_PIXY_RIGHT)==PIXY_INPUT_LEVEL) motion.cTurn(MOTION_TURN_RIGHT);
  if (digitalRead(PIN_PIXY_LEFT90)==PIXY_INPUT_LEVEL) motion.turnLeft90_inPlace();
  if (digitalRead(PIN_PIXY_RIGHT90)==PIXY_INPUT_LEVEL) motion.turnRight90_inPlace();
  if (digitalRead(PIN_PIXY_INTERSECT)==PIXY_INPUT_LEVEL) {
    pixyIntersectTimer=millis();
    serialMsg(MSG_INTER, 1);
    buzzer.beep(100);
    if (nextIntersectAction==DRIVE_NEXT_INTERSECT_STRAIGHT) {
      motion.fastForward();
      intersectActionTime=PIXY_DRIVE_INTERSECT_TIME_STRAIGHT;
    }
    else if (nextIntersectAction==DRIVE_NEXT_INTERSECT_LEFT) {
      motion.turnLeft90_inPlace();
      intersectActionTime=PIXY_DRIVE_INTERSECT_TIME_TURN;
      nextIntersectAction=DRIVE_NEXT_INTERSECT_STRAIGHT;
    }
    else if (nextIntersectAction==DRIVE_NEXT_INTERSECT_RIGHT) {
      motion.turnRight90_inPlace();      
      intersectActionTime=PIXY_DRIVE_INTERSECT_TIME_TURN;
      nextIntersectAction=DRIVE_NEXT_INTERSECT_STRAIGHT;
    }
  }
}
