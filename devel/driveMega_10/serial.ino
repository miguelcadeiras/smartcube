// showing info to the user
void showInfo()
{
  Serial.print( "megaDrive v.");
  Serial.println(ver);
}

#define SERIAL_STR_SPEED "sp"

#define SERIAL_BUFFER_SIZE  100
static uint8_t serialBufferIdx=0;
static unsigned long serialStringTimer=0;
static char serialBuffer[SERIAL_BUFFER_SIZE+1];

void serialLoop(){
  if (Serial.available()) {
    char ch = Serial.read();

    // ESPERANDO UNA STRING
      if (serialBufferIdx < SERIAL_BUFFER_SIZE) {
        if (ch!='\n' && ch!='\r') {
          serialBuffer[serialBufferIdx]=ch;
          serialBufferIdx++;
        }
        if (ch=='}') { 
          serialBuffer[serialBufferIdx]='\0';
          serialBufferIdx=0;
          serialParseString(serialBuffer);
        }
      }
      else {
        Serial.println ("STRING OVERFLOW");
        serialBufferIdx=0;
      }
      return;
  }

}

void serialParseString(char *buf) {
  char cmdSpr[SERIAL_BUFFER_SIZE];
  char cmdSpl[SERIAL_BUFFER_SIZE];

  //{spr:10.2;spl:2.22}
  //int s=sscanf (buf, "{sp%20[^;}];sp%20[^;}];}", cmdBuf, cmdArg);
  int s=sscanf (buf, "{spr:%20[^;}];spl:%20[^;}]}", cmdSpr, cmdSpl);
  if (s==2) {
    float spr=atof(cmdSpr);
    float spl=atof(cmdSpl);
    motion.runControl (spr, spl);
    return;
  }
  
  int stepperDisable=0;
  s=sscanf (buf, "{sdis:%d}", &stepperDisable);

  if (s==1) {
    if (stepperDisable==1)
      motion.disableMotors(1);
    else if (stepperDisable==0)
      motion.disableMotors(0);
    return;
  }

  int beepLen=0;
  int beepRep=0;
  s=sscanf (buf, "{\"beep\":[%d,%d]}", &beepLen, &beepRep);

  if (s==2) {
    //Serial.println (beepLen);
    //Serial.println (beepRep);
    buzzer.beep(beepLen, beepRep);
    return;
  }

  
}

void serialParseChar(char ch) {
 
}
