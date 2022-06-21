String ver= "nanoPixy_10";
#include <Pixy2.h>
#include "config.h"
#include "vector.h"


Pixy2 pixy;
#define defaultPixyCenter  (pixy.frameWidth/2)
#define maxPixyVectorSize pixy.frameHeight

int pixyWidth=0;
int pixyHeight=0;
int vectorsCount=0;
int intersectionCount=0;

PixyVector vect(79,52);

void setup(){
    Serial.begin(115200);
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
    delay(1000);
}

#define X 0
#define Y 1
char buf[200];
void loop(){
  pixy.line.getMainFeatures(PIXY_FEATURES_LINE_VECTOR | PIXY_FEATURES_LINE_INTERSECTION,false);
  int vectorsCount=pixy.line.numVectors;
  int intersectionCount=pixy.line.numIntersections;
  int vect0[2] = {-1, -1};
  int vect1[2] = {-1, -1};
  if (vectorsCount > 0) {
    vect0[X] = pixy.line.vectors->m_x0;
    vect0[Y] = pixy.line.vectors->m_y0;
    vect1[X] = pixy.line.vectors->m_x1;
    vect1[Y] = pixy.line.vectors->m_y1;
  }
  sprintf(buf, "{\"x0\":%d,\"y0\":%d,\"x1\":%d,\"y1\":%d,\"v\":%d,\"i\":%d}", vect0[X], vect0[Y], vect1[X], vect1[Y], vectorsCount, intersectionCount);
  Serial.println(buf);
  delay(25);
}
