
//designate interrupt pins
int PhotoSensor1Pin = ; //pin which photosensor 1 is connected too. Left blank for now
int PhotoSensor2Pin = ; //pin which photosensor 2 is connected too. Left blank for now

//bunch of variables for the photosensor interrupt.  Global for now but will update to pass them between functions
int CheckPhotoSensor = 0; //variable which will save last sensor detect
unsigned long PSStartTime; //time first photsensor is triggered
unsigned long V0; //initial velocity

void setup() {
  Serial.begin(115200);  // Set client serial freq to 115200

  //set interrupt pins to inputs
  pinMode(PhotoSensor1Pin, INPUT); //sets PhotoSesnor 1 to input
  attachInterrupt(digitalPinToInterrupt(PhotoSensor1Pin),CalcV0, CHANGE); //Sets up interrupt on change in PhotoSensor1Pin to run CalcV0
  
  pinMode(PhotoSensor2Pin, INPUT); //sets PhotoSesnor 2 to input
  attachInterrupt(digitalPinToInterrupt(PhotoSensor2Pin),CalcV0, CHANGE); //Sets up interrupt on change in PhotoSensor2Pin to run CalcV0
}

void loop() {
  Serial.println("Hello World");
  delay(1000);
}

void  CalcV0(){
  int PTDistance =; //distance between photosensors
  
  //checks if first photosensor triggered interrupt
  if(CheckPhotoSensor == 0){
    PSStartTime = millis(); //sets start time
    CheckPhotoSensor = 1;
  }

  //checks if second photosensor was triggered
  if(CheckPhotoSensor == 1){
    unsigned long PSTime = PSSStartTime -PSSStartTime; //calculates total time between Photo Sensor Triggers
    V0 = PTDistance / PStime; //calculates initial velocity
    CheckPhotoSensor = 0;  //ressets next trigger to be photosensor 1
  }
}
