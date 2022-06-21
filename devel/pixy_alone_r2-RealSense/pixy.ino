#define MAX_SPEED 100
#define LOW_SPEED 40

unsigned long pixyMaxSpeedTimer=0;

void processPixy() {
   bool noAction=false;
   
   pixy.line.getMainFeatures(PIXY_FEATURES_LINE_VECTOR | PIXY_FEATURES_LINE_INTERSECTION,false);
   vectorsCount=pixy.line.numVectors;
   intersectionCount=pixy.line.numIntersections;
   //pixy.line.vectors->print();

   if (vectorsCount >0) {
      lastVectorTime=millis();
      errorPrev=error;
  
          
      int vectX=pixy.line.vectors->m_x1;
      if (pixy.line.vectors->m_y1 > pixy.line.vectors->m_y0) {
        vectX=pixy.line.vectors->m_x0;
      }
      
      error = (int32_t) vectX - (int32_t) (pixyWidth/2)-1;
      //pid = (error*m_pgain + ((m_integral*m_igain)>>4) + (error - m_prevError)*m_dgain)>>10;
  
      // Integrador
      integral += error;
      
      if (integral>100)
        integral = 100;
      if (integral< -100)
        integral = -100;
  
      int32_t p = (error * pGain);
      int32_t i = (integral*iGain);
      int32_t d = (error - errorPrev) * dGain; 
      pid= p + i + d;
      
      // VELOCIDAD DEFAULT 100%
      if (millis()-pixyMaxSpeedTimer > 500) {
        speedLeft=MAX_SPEED;
        speedRight=MAX_SPEED;
      }
      else {
        speedLeft=LOW_SPEED;
        speedRight=LOW_SPEED; 
      }
      
        
      
  
      // SI EL VECTOR ES MUY CHICO NO CAMBIA NADA.
      if (abs(pixy.line.vectors->m_y1 - pixy.line.vectors->m_y0) < 30) {
        noAction=true;
        pixyMaxSpeedTimer=millis();
      }
  
      // SI EL ERROR ES MAYOR A 20. VEL=50%
      if (abs(error) > 20) {
        speedLeft=LOW_SPEED;
        speedRight=LOW_SPEED;        
        pixyMaxSpeedTimer=millis();
      } 
      
      // SI EL VECTOR ES CHICO (MENOR A LA MITAD) VEL=30%
      if (abs(pixy.line.vectors->m_y1 - pixy.line.vectors->m_y0) < (maxPixyVectorSize/2)+10) {
        speedLeft=LOW_SPEED;
        speedRight=LOW_SPEED;
        pixyMaxSpeedTimer=millis();
      }
  
      if (abs(pixy.line.vectors->m_x1 - pixy.line.vectors->m_x0) > 40) {
        speedLeft=LOW_SPEED;
        speedRight=LOW_SPEED;
        pixyMaxSpeedTimer=millis();
      }
    
      
      if (pid>0)
        speedRight=speedRight-abs(pid);
      else if (pid <0)
        speedLeft=speedLeft-abs(pid);
  
      /*if (speedLeft < -10)
        speedLeft = -10;
      if (speedRight < -10)
        speedRight=-10;
  */
  
      if (speedLeft <0)
        speedLeft = 0;
      if (speedRight <0)
        speedRight=0;

      if (lidarStop==true) {
        SerialS.println ("3;0;0;0");
        return;
      }
     
      if (noAction==false) {
        SerialS.print ("1;");
        SerialS.print (intersectionCount);
        SerialS.print (";");
        SerialS.print (speedRight);
        SerialS.print (";");
        SerialS.println (speedLeft);
      }
  
     
     
   }
   
   if (millis()-lastVectorTime > NO_LINE_TIMEOUT) {
        SerialS.println ("0;0;0;0");
   }
   /*vect.setCoordinates (pixy.line.vectors->m_x0, pixy.line.vectors->m_y0, pixy.line.vectors->m_x1, pixy.line.vectors->m_y1);
   Serial.print ("Len: ");
   Serial.print (vect.getLength());
   Serial.print (" Ang: ");
   Serial.print (vect.getAngle());
   Serial.print (" Off: ");
   Serial.print (vect.getOffset());
   
   Serial.println();*/

  
}
