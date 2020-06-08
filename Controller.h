#ifndef Controller_h
#define Controller_h

#include <Encoder.h>
#include <Path.h>
#include <Photo.h>

#define CT_PER_M 9947.183943  // Encoder counter per meter

// Current mode while running
enum Mode {
  CALIBRATE,
  DESCEND,
  RUN,
  WAIT
} currentMode;

HardwareTimer sampleTimer(TIM3); // Timer for controller interrupt
double y, y_goal, controllerEffort;

// Default user values
double y_zero = 0.0;
double limMaxEffort = 2.0;
double K_adj = 0.1;
double soften_adj = 1.0;

// Logging
#define FRAMES 500
float y_arr[FRAMES];
float y_goal_arr[FRAMES];
float controllerEffort_arr[FRAMES];

// Controller biquad
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

// Cascade from ME477
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

// Run the controller biquad and return controller effort
double getControllerEffort(double error){
  return cascade(error, controllerBiquad, 1);
}

// Calculate the current location. Account for calibration
double getY(){
  unsigned int encCount = readEncoder();
  return encCount / CT_PER_M - y_zero;
}

// ISR run at 1000Hz
void callback(HardwareTimer *timer){
  y = getY();

  // Switch between run mode behaviors
  switch (currentMode){

    // Normal controller that follows goal path
    case RUN:
      y_goal = path.getTargetPos();
      if (y_goal < 0){
        y_goal = 0;
      } else if (y_goal > H_PS2){
        y_goal = H_PS2;
      }
      break;

    // Initial mode that moves up slowly until it reaches
    // the lower photointerrupter and records location.
    case CALIBRATE:
      if (digitalRead(PS_PIN2) == LOW){
        y_zero = y - H_PS2 + H_CAL;
        currentMode = DESCEND;
        y_goal = H_PS2 - H_CAL;
      } else {
        y_goal += 3.0e-5;
      }
      break;

    // After calibration descend slowly to y=0m before continue
    case DESCEND:
      if (y_goal > 10.0e-5){
        y_goal -= 10.0e-5;
      } else {
        y_goal = 0.0;
        currentMode = WAIT;
      }
      break;

    // No nothing allowing for user control. Limit goal range.
    case WAIT:
      if (y_goal < 0){
        y_goal = 0;
      } else if (y_goal > H_PS2){
        y_goal = H_PS2;
      }
      break;
  }

  // Get error and run controller
  double err = y_goal - y;
  controllerEffort = getControllerEffort(err) * K_adj;

  // Soften controller gain after egg in caught
  if (path.currTime > path.t2) controllerEffort *= soften_adj;

  // Limit controller effort
  if (controllerEffort > limMaxEffort) controllerEffort = limMaxEffort;
  if (controllerEffort < -limMaxEffort) controllerEffort = -limMaxEffort;

  // If one of the first 500 sample, record
  if (idx < FRAMES){
    y_arr[idx] = y;
    y_goal_arr[idx] = y_goal;
    controllerEffort_arr[idx] = controllerEffort;
    idx++;
  }

  // Output control effort to motor amp
  dac.write(controllerEffort);
}

// Configure controller timer for 1000Hz and ISR on compare interrupt
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
