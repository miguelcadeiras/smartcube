// showing info to the user
void showInfo()
{
  Serial.print( "SmartCubeDrive v.");
  Serial.println(ver);
  Serial.println("w,s,a d  | FORWARD,BACKWARDS, LEFT,RIGHT");
  Serial.println("z,c      | TURN IN PLACE LEFT / RIGHT");
  Serial.println("f        | FAST FORWARD");
  Serial.println("0        | STOP");
  Serial.println("q        | STOP AUTO DRIVE");
  Serial.println("g        | EMERGENCY STOP RESET");
  Serial.println("p        | FOLLOW LINE");
  Serial.println("r        | REALSENSE");

  Serial.println("t        | Z AXIS INFO");
  Serial.println("u        | Z AXIS INFO");
  Serial.println("i,o      | PRINT STATUS INFO, PRINT SPEED ");
  
  Serial.println("m,n      | INCREASE,DECREASE Default Speed");
  Serial.println("L,R,D    | NEXT INTERSECT: LEFT,RIGHT");
  Serial.println("1,2,3    | RESET: DefaultSpeed, Center Drive, Encoder");
  Serial.println("4,5,6    | US SENSOR TOGGLE, NO ENCODER TOGGLE, TILT TOGGLE"); 

  Serial.println("e        | RETURNS ENCODER POSITION SINCE RESET"); 
  Serial.println("E        | RETURNS MOTORS POSITION SINCE RESET"); 
  Serial.println("v,b      | VERVOSE MODE,VERBOSE DRIVE MODE ");
  Serial.println("x        | READ BATTERIES VOLTAG (24v & 36v)");
 
  Serial.println("7,8,9  |aTurn, bturn,cTurn right");
  Serial.println("/,*,-  |aTurn, bturn,cTurn left");
  Serial.println("?        | Help");


}

#define SERIAL_STR_SPEED "sp"

#define SERIAL_WAIT_STRING 1
#define SERIAL_WAIT_CHAR 0
#define SERIAL_BUFFER_SIZE  30
static int serialMode=SERIAL_WAIT_CHAR;
static uint8_t serialBufferIdx=0;
static unsigned long serialStringTimer=0;
static char serialBuffer[SERIAL_BUFFER_SIZE+1];

void serialLoop(){
  // Si pasa mas de cierto tiempo sin terminar la string vuelve al modo CHAR
  if (serialMode==SERIAL_WAIT_STRING && millis()-serialStringTimer > 1000) {
    Serial.println ("STRING TIMEOUT");
    serialMode=SERIAL_WAIT_CHAR;
  }
  
  if (Serial.available()) {
    char ch = Serial.read();

    // ESPERANDO UNA STRING
    if (serialMode==SERIAL_WAIT_STRING) {
      serialStringTimer=millis();
      if (serialBufferIdx < SERIAL_BUFFER_SIZE) {
        serialBuffer[serialBufferIdx]=ch;
        serialBufferIdx++;
        if (ch=='}') { 
          serialBuffer[serialBufferIdx]='\0';
          serialParseString(serialBuffer);
          serialMode=SERIAL_WAIT_CHAR;
        }
      }
      else {
        Serial.println ("STRING OVERFLOW");
        serialMode=SERIAL_WAIT_CHAR;
      }
      return;
    }
    
    // ESPERANDO UN CHAR
    else if (serialMode==SERIAL_WAIT_CHAR) { 
      serialParseChar(ch);
    }
  } 

}

void serialParseString(char *buf) {
  char cmdBuf[SERIAL_BUFFER_SIZE];
  char cmdArg[SERIAL_BUFFER_SIZE];

  int s=sscanf (buf, "{%20[^;}];%20[^;}];}", cmdBuf, cmdArg);
 
  if (s==2 && strcmp (cmdBuf, SERIAL_STR_SPEED) == 0) {
    int newSpeed=0;
    if (sscanf (cmdArg, "%d", &newSpeed))
       motion.setSpeed(newSpeed);
       Serial.print("OK Speed set: ");
       Serial.println(newSpeed);
  }

  
}

void serialParseChar(char ch) {
  switch(ch){
      //verboseMode
      case 'v':
        if(verbose){
          verbose=false;
          Serial.println("Verbose mode OFF");
        }
        else{
          verbose=true;
          Serial.println("Verbose mode ON");
        }
        break;
      //verboseDrive
        case 'b':
        if(motion.verbose){
          motion.verbose=false;
          Serial.println("Verbose DRIVE mode OFF");
        }
        else{
          motion.verbose=true;
          Serial.println("Verbose DRIVE mode ON");
        }
        break;
        
      //move Forward  
      case  'f':
        motion.fastForward();
        Serial.println( "moving FastForward");
        break;
      case 'w':
        motion.forward();
        Serial.println("moving Forward");
        break;
      //move Backwards
      case 's':
        if (motion.getStatus() == MOTION_PAUSED)
          motion.resume();
        motion.backwards();
        Serial.println("moving BackWards");
        break;
      //move left
      case 'a':
        motion.turnLeft90();
        Serial.println( "Turning Left"); 
        break;
      //move right
      case 'd':
        motion.turnRight90();
        Serial.println( "Turning Right"); 
        break;
      //move left
      case 'z':
        motion.turnLeft90_inPlace();
        Serial.println( "Turning Left In Place"); 
        break;
      //move right
      case 'c':
        motion.turnRight90_inPlace();
        Serial.println( "Turning Right in Place"); 
        break;
        
      // STOP
      case '0':
        driveMode=DRIVE_MANUAL;
        motion.stop();
        break;
      // STOP AUTO DRIVE
      case 'q':
        driveMode=DRIVE_MANUAL;
        motion.stop();
        break;
      case 'g':
        buzzer.beep(100);
        driveMode=DRIVE_MANUAL;
        motion.resetEmergency();
        Serial.println("Emergency Reset");
        break;
      // PIXI DRIVE
      case 'p':
        driveMode = DRIVE_AUTO;
        Serial1.print('p');
        buzzer.beep(250);
        Serial.println("Pixy Drive Activated...");
        break;
      // REALSENSE
      case 'r':
        driveMode = DRIVE_AUTO;
        Serial1.print('r');
        buzzer.beep(700);
        Serial.println("Realsense Drive Activated...");
        break;
      case 't':
        //Serial.print (tiltSensor.getZ());
        serialMsg (MSG_ZANGLE, tiltSensor.getZFloat());
        break;


        
      // SETTINGS
      //Increase Default Speed by 100
      case 'm':
          motion.setSpeed (motion.getSpeed()+100);
          Serial.print("Default Speed: ");
          Serial.println(motion.getSpeed());
          break;
      //Decrease Default Speed by 100
      case 'n':
          motion.setSpeed (motion.getSpeed()-100);
          Serial.print("Default Speed: ");
          Serial.println(motion.getSpeed());
          break;
      //DEFAULT TURN SET TO RIGHT
      case 'D':
          nextIntersectAction=DRIVE_NEXT_INTERSECT_STRAIGHT;
          break;
      case 'L':
          nextIntersectAction=DRIVE_NEXT_INTERSECT_LEFT;
          break;
      //DEFAULT TURN SET TO LEFT
      case 'R':
          nextIntersectAction=DRIVE_NEXT_INTERSECT_RIGHT;
          break;
        
  
      //RESETS
      case '1':
          motion.setSpeed(STEPPER_DEFAULT_SPEED);
          Serial.println("SPEED RESET to Factory");
          break;
  
        case '3':
          encoder.resetDistance();
          Serial.println("ENCODER RESET");
          break;
    
        case '4':
          ultrasoundDetection=not(ultrasoundDetection);
          Serial.print ("Ultrasound Detection: ");
          Serial.println (ultrasoundDetection);
          break;
        case '5':
          noEncoderDetection=not(noEncoderDetection);
          Serial.print ("No Encoder Detection: ");
          Serial.println (noEncoderDetection);
          break;
      case '6':
          tiltDetection=not(tiltDetection);
          Serial.print ("Tilt Detection: ");
          Serial.println (tiltDetection);
          break;
          
        // HELP
        case '?':
          showInfo();
          break;
  
        //Test Kart Connection
        case  'K':
          Serial.println("KART CONNECTION SUCCESS");
          break;
        case  'e':
          Serial.println (encoder.getMts());
          break;
        case 'E':
          //serialMsg (MSG_ZANGLE, tiltSensor.getZFloat());
          Serial.println("Motors position:");
          Serial.println (motion.getMotorPosition(STEPPER_RIGHT));
          Serial.println (motion.getMotorPosition(STEPPER_LEFT));
          break;

        // Batteries voltage
        case 'x':
          serialMsg (MSG_VMOT, vccMotor.read());
          serialMsg (MSG_VCOMP, vccComp.read());
          break;
        case  'i':
          Serial.print("motion: ");
          Serial.println( motion.getStatus() );
          Serial.print("accel: ");
          Serial.println( motion.getAccel() );
          Serial.print("speed: ");
          Serial.println( motion.getSpeed() );
          Serial.print("speed R: ");
          Serial.println( motion.getStepperSpeed(STEPPER_RIGHT));
          Serial.print("speed L: ");
          Serial.println( motion.getStepperSpeed(STEPPER_LEFT));
          Serial.println();
          Serial.print("usSensor1: ");
          Serial.println(usSensor1.getDistance());
          Serial.print("usSensor2: ");
          Serial.println(usSensor2.getDistance());
          
          Serial.println ("-- PIXY INPUTS:");
          Serial.print( "STRAIGHT Pin:");
          Serial.println(digitalRead(PIN_PIXY_STRAIGHT));
          Serial.print( "LEFT Pin:");
          Serial.println(digitalRead(PIN_PIXY_LEFT));
          Serial.print( "RIGHT Pin:");
          Serial.println(digitalRead(PIN_PIXY_RIGHT));
          Serial.print( "LEFT 90 Pin:");
          Serial.println(digitalRead(PIN_PIXY_LEFT90));
          Serial.print( "RIGHT 90 Pin:");
          Serial.println(digitalRead(PIN_PIXY_RIGHT90));
          Serial.print( "STOP Pin:");
          Serial.println(digitalRead(PIN_PIXY_STOP));
        break;
        case '7':
          
          motion.turnLeft90_inPlace();
          Serial.println( "Turning Left In Place");

          motion.forward();
          Serial.println("moving forward");
          
          motion.turnLeft90_inPlace();
          Serial.println( "Turning Left In Place");
          encoder.resetDistance();

          

          break;
        case '8':
          motion.bTurn(MOTION_TURN_RIGHT);
          Serial.println("bTurn RIGHT");
          break;
        case '9':
          motion.cTurn(MOTION_TURN_RIGHT);
          Serial.println("cTurn RIGHT");
          break;
        case '/':
          motion.aTurn(MOTION_TURN_LEFT);
          Serial.println("aTurn(LEFT)");
          break;
        case '*':
          motion.bTurn(MOTION_TURN_LEFT);
          Serial.println("bTurn(LEFT)");
          break;
        case '-':
          motion.cTurn(MOTION_TURN_LEFT);
          Serial.println("cTurn(LEFT)");
          break;
       case 'o':
          Serial.print("speed:");
          Serial.println(motion.getSpeed());
          break;
       case '{': 
          serialMode=SERIAL_WAIT_STRING;
          serialStringTimer=millis();
          serialBuffer[0]='{';
          serialBufferIdx=1;
          break;   
        default:
          break;
    }
}
