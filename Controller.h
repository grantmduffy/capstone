#ifndef Controller_h
#define Controller_h

#include <Encoder.h>
#include <Path.h>
#include <Photo.h>

HardwareTimer sampleTimer(TIM3);
double y, y_goal, controllerEffort;

struct biquad {
  double b0; double b1; double b2;  // numerator
  double a0; double a1; double a2;  // denominator
  double x0; double x1; double x2;  // input
  double y1; double y2;};           // output
static struct biquad controllerBiquad[] = {{
  4836.780688, -4549.787648, 0.000000, 
  1.000000, -0.711621, 0.000000, 
  0, 0, 0, 0, 0
}};

double cascade( double xin,        // input
        struct biquad *fa,         // biquad array
        int ns){                   // no. segments
  struct biquad* f = fa;
  f->x0 = xin;
  double y0;
  int i;
  for(i = 0; i < ns; i++){
    // passes previous values
    if ( i > 0){
      f->x0 = y0;
    }
    // calculate y0
    y0 = (f->b0*f->x0 + f->b1*f->x1 + f->b2*f->x2 - f->a1*f->y1 - f->a2*f->y2)/f->a0;
    //update x and y
    f->x2 = f->x1; f->x1 = f->x0;
    f->y2 = f->y1; f->y1 = y0;
    f++;  // increment f
  }
  return y0;
};

double getControllerEffort(double error){
  return cascade(error, controllerBiquad, 1);
}

void callback(HardwareTimer *timer){
  unsigned int encCount = readEncoder();
  y = encCount / 10185.91636;
  y_goal = path.getTargetPos();
//  y_goal += 0.5;
  double err = y_goal - y;
  controllerEffort = getControllerEffort(err) / 10;
  if (controllerEffort > 10.0) controllerEffort = 10.0;
  if (controllerEffort < -10.0) controllerEffort = -10.0;
  dac.write(map(controllerEffort, -10, 10, 0, 4095));
}

void setupController(){
  sampleTimer.pause(); 
  sampleTimer.setMode(1, TIMER_OUTPUT_COMPARE);
  sampleTimer.setPrescaleFactor(36000);  // Divide to 2kHz
  sampleTimer.setOverflow(2);            // Divide to 1kHz
  sampleTimer.setCaptureCompare(1, 1);
  sampleTimer.attachInterrupt(1, callback);
  sampleTimer.refresh();
  sampleTimer.resume();
}


#endif
