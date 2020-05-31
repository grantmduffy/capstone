#include <math.h>

// Design Constants
#define R 0.032               // radius [m]
#define W_FREE 401.076662108  // free velocity
#ifndef Path_h
#define Path_h

#define G 9.81                // gravitational constant
#define TAU 0.128738374689150 // time constant

#define T1_POW 2
#define T2_POW 3
#define T3_POW 4

class Path {
  public:
    void begin(double initV, unsigned long stime);
    double getTargetPos();
    double t1, t2, t3, v0;
    unsigned long psTime2;
  private:
    const double P_T1[T1_POW + 1] = {1.8933799e-03,  -1.4144048e-02,   7.7584335e-02};
    const double P_T2[T2_POW + 1] = {1.4825302e-03,  -1.1955975e-02,  -1.9359523e-02,   2.7819310e-01};
    const double P_T3[T3_POW + 1] = {3.5350856e-03,  -2.5280737e-02,   9.6704519e-02,  -2.5276310e-01,   5.1297253e-01};
};


void Path::begin(double initV, unsigned long stime){
  v0 = initV;
  psTime2 = stime;
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

double Path::getTargetPos(){
  double fact = -1.0667 * v0 + 4;
  double tau2 = TAU * fact;
  double aConst = -(v0 + G *t2) / (t2 - t3);
  double currTime = ((double) (micros() - psTime2)) / 1000000;
  double pos;
  if (currTime < t1){
    pos = R * W_FREE * (currTime + TAU * exp(-currTime /TAU) - TAU);
  } else if(currTime < (2 * t1)){
    pos = 2 * (R * W_FREE * (t1 + TAU * exp(-t1 / TAU) - TAU)) - (R * W_FREE * (((2 * t1) - currTime) + TAU * exp(-((2 * t1) - currTime) / TAU) - TAU));
  } else if(currTime < t2){
    pos = 2 * (R * W_FREE * (t1 + TAU * exp(-t1 / TAU) - TAU)) - (R * W_FREE * ((currTime - 2 * t1) + tau2 * exp(-(currTime - 2 * t1) / tau2) - tau2)); //F2
  } else if (currTime < t3){
    pos = .5 * aConst * (currTime - t3) * (currTime - t3);
  } else{
    pos = 0;
  }
  return pos;
}

Path path;

#endif
