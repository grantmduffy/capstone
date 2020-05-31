#ifndef Photo_h
#define Photo_h

#define PS_PIN1 1         // pin which photosensor 1 is connected to. Left blank for now TODO: Correct pin
#define PS_PIN2 2         // pin which photosensor 2 is connected to. Left blank for now TODO: Correct pin
#define PS_DISTANCE 1000  // distance between the photosensors, unit in mm. TODO: Correct the distance

class Photo {
  public:
    static unsigned long psTime1;    // time first photsensor is triggered
    static unsigned long psTime2;    // initial velocity
    static double vel;
    void begin();
    double calcVelocity();
  private:
    static void interrupt1();
    static void interrupt2();
};

void Photo::begin(){
  pinMode(PS_PIN1, INPUT); //sets PhotoSesnor 1 to input
  attachInterrupt(digitalPinToInterrupt(PS_PIN1), interrupt1, RISING); //Sets up interrupt on change in PhotoSensor1Pin to run CalcV0
  pinMode(PS_PIN2, INPUT); //sets PhotoSesnor 2 to input
  attachInterrupt(digitalPinToInterrupt(PS_PIN2), interrupt2, RISING); //Sets up interrupt on change in PhotoSensor2Pin to run CalcV0
}

void Photo::interrupt1(){
  psTime1 = micros();
}

void Photo::interrupt2(){
  psTime2 = micros();
}

double Photo::calcVelocity(){
  vel = PS_DISTANCE / (psTime2 - psTime1);
  return vel;
}

#endif
