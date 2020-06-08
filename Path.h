#include <math.h>

#ifndef Path_h
#define Path_h

// Design Constants
#define R 0.032               // radius [m]
#define W_FREE 401.076662108  // free velocity
#define H_PS2 0.6             // height of photosensor 2 from 0
#define G 9.81                // gravitational constant
#define TAU 0.128738374689150 // time constant
#define H_CAL 0.05

// Powers of lookup time polynomials
#define T1_POW 2
#define T2_POW 3
#define T3_POW 4

// Class for calculating trajectory
class Path {
  public:
    void begin(double initV);
    double getTargetPos();
    double t1, t2, t3, v0;
    unsigned long psTime2;
    double currTime;
  private:
    // Coefficients for lookup time polynomials
    const double P_T1[T1_POW + 1] = {1.9047313e-03,  -1.4154239e-02,   7.1531688e-02};
    const double P_T2[T2_POW + 1] = {1.5281608e-03,  -1.2411495e-02,  -1.4831307e-02,   2.5422850e-01};
    const double P_T3[T3_POW + 1] = {4.3747360e-03,  -3.0438227e-02,   1.0921664e-01,  -2.6453495e-01,   4.8110713e-01};
};

// Mark the begging of a trajectory. Called when egg at second photosensor
void Path::begin(double initV){
  v0 = initV;
  psTime2 = micros(); // mark start time

  // Calculate t1, t2, t3. Equivilent to polyval in MATLAB
  t1 = 0; t2 = 0; t3 = 0;
  for (int i = 0; i <= T1_POW; i++){
    t1 += P_T1[i] * pow(v0, T1_POW - i);
  }
  for (int i = 0; i <= T2_POW; i++){
    t2 += P_T2[i] * pow(v0, T2_POW - i);
  }
  for (int i = 0; i <= T3_POW; i++){
    t3 += P_T3[i] * pow(v0, T3_POW - i);
  }
}

// Get the target position at any time
double Path::getTargetPos(){
  double fact = -1.0667 * v0 + 4;  // Get slowing factor for f2
  double tau2 = TAU * fact;        // Slowed tau
  double aConst = -(v0 + G *t2) / (t2 - t3);  // Constant accel after catch
  currTime = ((double) (micros() - psTime2)) / 1000000;
  double pos;

  // Piecewise functions of position from MATLAB
  if (currTime < t1){ // f1
    pos = R * W_FREE * (currTime + TAU * exp(-currTime /TAU) - TAU);
  } else if(currTime < (2 * t1)){ // f5
    pos = 2 * (R * W_FREE * (t1 + TAU * exp(-t1 / TAU) - TAU)) - (R * W_FREE * (((2 * t1) - currTime) + TAU * exp(-((2 * t1) - currTime) / TAU) - TAU));
  } else if(currTime < t2){  // f2
    pos = 2 * (R * W_FREE * (t1 + TAU * exp(-t1 / TAU) - TAU)) - (R * W_FREE * ((currTime - 2 * t1) + tau2 * exp(-(currTime - 2 * t1) / tau2) - tau2)); //F2
  } else if (currTime < t3){  // f3
    pos = .5 * aConst * (currTime - t3) * (currTime - t3);
  } else{  // After path concluded
    pos = 0;
  }
  return pos;
}

Path path;

#endif
