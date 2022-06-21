#include "vector.h"

PixyVector::PixyVector (int w, int h) {
  width=w;
  height=h;
  center=width/2;
  x0=0;
  y0=0;
  x1=0;
  y1=0;
  valid=false;
}

void PixyVector::setCoordinates(int xStart, int yStart, int xEnd, int yEnd) {
  x0=xStart;
  y0=yStart;
  x1=xEnd;
  y1=yEnd;
  calculate();
}

void PixyVector::calculate() {
  length = sqrt (float((x0-x1) * (x0-x1)) + float((y0-y1) * (y0-y1))); 
  angle = atan(float (y0-y1) / float (x0-x1))  ;
  angle = angle * 57.2958;
  offset = center-x0;
}
