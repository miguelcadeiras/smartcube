#ifndef VECTOR_H
#define VECTOR_H

#include "arduino.h"

class PixyVector {
  private:
    int width;
    int height;
    int center;
    int x0,y0,x1,y1;
    bool valid=false;
    float angle;
    float length;
    int offset=0;
    int direction=0;
    void calculate();
  public:
    void setCoordinates(int xStart, int yStart, int xEnd, int yEnd);
 
    float getAngle () {return angle;};
    float getLength() {return length;};
    int getOffset() {return offset;}
    
    PixyVector (int w, int h);
  
};

#endif
