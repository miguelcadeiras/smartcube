#define SERIAL_BUFFER_SIZE  30
char serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIdx=0;
unsigned long realSenseDetectedTimer=0;

void processRealSense() {
   int err;
   int cmd;

   int l=sscanf (serialBuffer, "{LD:%d}", &cmd);
    if (l==1) {
      Serial.println ("RECEIVED_LD_STOP");
      lidarStop=true;
      lidarStopTimer=millis();
    }
   
   int s=sscanf (serialBuffer, "{%d}", &err);
   
   if (driveMode==DRIVE_MODE_PIXY_ONLY) {
    Serial.println ("DRIVE_MODE_PIXY_ONLY");
    return;
   }

   if (s==1) {
    errorPrev=error;
    error=err/10;

    if (abs(error) > 25)
      realSenseDetectedTimer=millis();
    
    // Detecto barra por masde 1000ms se pasa a modo realsense;
    if (driveMode==DRIVE_MODE_PIXY && abs(error) <= 25 && millis()-realSenseDetectedTimer > 250)
      driveMode=DRIVE_MODE_REALSENSE;
   
    if (driveMode==DRIVE_MODE_PIXY) {
      Serial.println ("DRIVE_MODE_PIXY");
      return;
    }
    Serial.println ("DRIVE_MODE_PIXY_REALSENSE");
    
    if (abs(error) > 60) {
      driveMode=DRIVE_MODE_PIXY;
      return;
    }

    int32_t p = (error * pGain);

    int32_t i = (integral*iGain);
    int32_t d = (error - errorPrev) * dGain; 
    pid= p + i + d;
    
    // VELOCIDAD DEFAULT 100%
    speedLeft=100;
    speedRight=100;

    // SI EL ERROR ES MAYOR A 20. VEL=50%
    if (abs(error) > 10) {
      speedLeft=50;
      speedRight=50;        
    } 
    
    if (pid>0)
      speedRight=speedRight-abs(pid);
    else if (pid <0)
      speedLeft=speedLeft-abs(pid);



    if (speedLeft <0)
      speedLeft = 0;
    if (speedRight <0)
      speedRight=0;
    if (lidarStop==true)
      SerialS.print ("3;");
    else
      SerialS.print ("2;");
    SerialS.print (intersectionCount);
    SerialS.print (";");
    SerialS.print (speedRight);
    SerialS.print (";");
    SerialS.println (speedLeft);
   }
  
}
void realSenseFlushSerial() {
  if (Serial.available()) 
    char ch=Serial.read();
}
bool realSenseReadSerial() {
  
  if (Serial.available()) {
    char ch=Serial.read();
    //SerialS.write (ch);    
    if (serialBufferIdx < SERIAL_BUFFER_SIZE-1) {
      serialBuffer[serialBufferIdx]=ch;
      serialBufferIdx++;
      if (ch=='\n') { 
        serialBuffer[serialBufferIdx]='\0';
        serialBufferIdx=0;
        processRealSense();
      }
    }
    else {
      serialBufferIdx=0;
    }
  } 
}
