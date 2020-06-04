#ifndef Controller_h
#define Controller_h

#include <Encoder.h>
#include <Path.h>
#include <Photo.h>

#define CT_PER_M 9947.183943

enum Mode {
  CALIBRATE,
  DESCEND,
  RUN,
  WAIT
};

HardwareTimer sampleTimer(TIM3);
double y, y_goal, controllerEffort;
double y_zero = 0.0;
Mode currentMode;
bool limitEffort = true;
double maxEffort = 10.0;
double limMaxEffort = 2.0;
double K_adj = 0.1;

#define FRAMES 500
unsigned int idx;
float y_arr[FRAMES];
float y_goal_arr[FRAMES];
float controllerEffort_arr[FRAMES];

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

double getY(){
  unsigned int encCount = readEncoder();
  return encCount / CT_PER_M - y_zero;
}

void callback(HardwareTimer *timer){
  y = getY();

  switch (currentMode){
    case RUN:
      y_goal = path.getTargetPos();
      if (y_goal < 0){
        y_goal = 0;
      } else if (y_goal > H_PS2){
        y_goal = H_PS2;
      }
      break;
    case CALIBRATE:
      if (digitalRead(PS_PIN2) == LOW){
        y_zero = y - H_PS2;
        currentMode = DESCEND;
        y_goal = H_PS2;
      } else {
        y_goal += 3.0e-5;
      }
      break;
    case DESCEND:
      if (y_goal > 10.0e-5){
        y_goal -= 10.0e-5;
      } else {
        y_goal = 0.0;
        currentMode = WAIT;
      }
      break;
    case WAIT:
      if (y_goal < 0){
        y_goal = 0;
      } else if (y_goal > H_PS2){
        y_goal = H_PS2;
      }
      break;
  }
  
  double err = y_goal - y;
  controllerEffort = getControllerEffort(err) * K_adj;
  
  if (limitEffort){
    if (controllerEffort > limMaxEffort) controllerEffort = limMaxEffort;
    if (controllerEffort < -limMaxEffort) controllerEffort = -limMaxEffort;
  }
  if (controllerEffort > maxEffort) controllerEffort = maxEffort;
  if (controllerEffort < -maxEffort) controllerEffort = -maxEffort;

  if (idx < FRAMES){
    y_arr[idx] = y;
    y_goal_arr[idx] = y_goal;
    controllerEffort_arr[idx] = controllerEffort;
    idx++;
  }
  
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
  y_goal = getY();
}


#endif
