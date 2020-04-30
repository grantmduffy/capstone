
//designate interrupt pins
#define PS_PIN1 = 1; //pin which photosensor 1 is connected too. Left blank for now TODO: Correct pin
#define PS_PIN2 = 2; //pin which photosensor 2 is connected too. Left blank for now TODO: Correct pin

//bunch of variables for the photosensor interrupt.  Global for now but will update to pass them between functions
unsigned long ps_time_1; //time first photsensor is triggered
unsigned long ps_time_2; //initial velocity
unsigned long t1, t2;

void setup() {
  Serial.begin(115200);  // Set client serial freq to 115200

  //set interrupt pins to inputs
  pinMode(PhotoSensor1Pin, INPUT); //sets PhotoSesnor 1 to input
  attachInterrupt(digitalPinToInterrupt(PhotoSensor1Pin), CalcV0, RISING); //Sets up interrupt on change in PhotoSensor1Pin to run CalcV0
  
  pinMode(PhotoSensor2Pin, INPUT); //sets PhotoSesnor 2 to input
  attachInterrupt(digitalPinToInterrupt(PhotoSensor2Pin), CalcV0, RISING); //Sets up interrupt on change in PhotoSensor2Pin to run CalcV0
}

void loop() {
  while (ps_time_1 == 0 && ps_time_2 == 0){}
  calc_velocity();
  lookup_times();

//  (controller loop)

  t1 = 0;
  t2 = 0;
}

void interrupt_1(){
  
}

void interrupt_2(){
  
}

void calc_velocity(){
  
}

void lookup_times(){
  
}

unsigned int get_target_position(unsigned long t1, unsigned long t2, unsigned long t){
  
}

//void  CalcV0(){
//  int PTDistance = 1000; //distance between photosensors
//  
//  //checks if first photosensor triggered interrupt
//  if(CheckPhotoSensor == 0){
//    PSStartTime = millis(); //sets start time
//    CheckPhotoSensor = 1;
//  }
//
//  //checks if second photosensor was triggered
//  if(CheckPhotoSensor == 1){
//    unsigned long PSTime = PSSStartTime - PSSStartTime; //calculates total time between Photo Sensor Triggers
//    V0 = 1000 * PTDistance / PStime; //calculates initial velocity
//    CheckPhotoSensor = 0;  //ressets next trigger to be photosensor 1
//  }
//}
