#ifndef Photo_h
#define Photo_h

#include <Controller.h>
#include <Path.h>

#define PS_PIN1 PB14
#define PS_PIN2 PB15
#define PS_DISTANCE 0.034

unsigned long psTime1, psTime2;
double v0;

double calcVelocity(){
  double deltaT = double(psTime2 - psTime1) / 1000000.0;
  v0 = PS_DISTANCE / deltaT + 0.5 * G * deltaT;
  return v0;
}

void interrupt1(){
  psTime1 = micros();
}

void interrupt2(){
  psTime2 = micros();
  path.begin(calcVelocity());
}

void setupPhotointerrupters(){
  pinMode(PS_PIN1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PS_PIN1), interrupt1, RISING); //Sets up interrupt on change in PhotoSensor1Pin to run CalcV0
  pinMode(PS_PIN2, INPUT);
  attachInterrupt(digitalPinToInterrupt(PS_PIN2), interrupt2, RISING); //Sets up interrupt on change in PhotoSensor2Pin to run CalcV0
}

#endif
