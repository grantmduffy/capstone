#include <Wire.h>

// Designate interrupt pins
#define PS_PIN1 = 1; //pin which photosensor 1 is connected too. Left blank for now TODO: Correct pin
#define PS_PIN2 = 2; //pin which photosensor 2 is connected too. Left blank for now TODO: Correct pin
#define PS_Distance = 1000; // distance between the photosensors, unit in mm. TODO: Correct the distance
#define Lookup_Table_Size = 256;

// Bunch of variables for the photosensor interrupt.  Global for now but will update to pass them between functions
unsigned long ps_time_1; //time first photsensor is triggered
unsigned long ps_time_2; //initial velocity
unsigned long t1, t2;
double vel;

// Parameters for I2C
#define DAC_I2C_ADDRESS 0x60
#define DAC_WRITE_COMMAND 0x40

void setup() {
  Serial.begin(115200);  // Set client serial freq to 115200
  Wire.begin();          // Start I2C for DAC

  //set interrupt pins to inputs
  pinMode(PS_PIN1, INPUT); //sets PhotoSesnor 1 to input
  attachInterrupt(digitalPinToInterrupt(PS_PIN1), interrupt_1, RISING); //Sets up interrupt on change in PhotoSensor1Pin to run CalcV0
  
  pinMode(PS_PIN2, INPUT); //sets PhotoSesnor 2 to input
  attachInterrupt(digitalPinToInterrupt(PS_PIN2), interrupt_2, RISING); //Sets up interrupt on change in PhotoSensor2Pin to run CalcV0
}

void loop() {
  while (ps_time_1 == 0 && ps_time_2 == 0){}
  double initial_velocity  = calc_velocity();
  lookup_times();

//  (controller loop)

//  t1 = 0;
//  t2 = 0;
}

void interrupt_1(){
  ps_time_1 = mills();
}

void interrupt_2(){
  ps_time_2 = mills();
}

double calc_velocity(){
  vel = PS_Distance / (ps_time_2 - ps_time_1);
  return vel;
}

void lookup_times(double initial_velocity){
  for(i=0; i<Lookup_Talbe_Size;i++){
    if(Lookup_Table[i,0] == initial_velocity){
      t1 = Lookup_Table[i,1];
      t2 = Lookup_Table[i,2];
      t3 = Lookup_Table[i,3]
    } else if(Lookup_Table[i+1,0] > initial_velocity && Lookup_Table[i,0]< initial_velocity){
      t1 = (Lookup_Table[i+1,1] - Lookup_Table[i,1]) / (Loopup_Table[i+1,0] - Loopup_Table[i,0]) * (initial_velocity - Lookup_Table[i,0]) + Lookup_Table[i,1]
      t2 = (Lookup_Table[i+1,2] - Lookup_Table[i,2]) / (Loopup_Table[i+1,0] - Loopup_Table[i,0]) * (initial_velocity - Lookup_Table[i,0]) + Lookup_Table[i,2]
      t3 = (Lookup_Table[i+1,3] - Lookup_Table[i,3]) / (Loopup_Table[i+1,0] - Loopup_Table[i,0]) * (initial_velocity - Lookup_Table[i,0]) + Lookup_Table[i,3]
    }
  }
  return t1,t2,t3;
}

unsigned int get_target_position(unsigned long t1, unsigned long t2, unsigned long t){
  
}

void write_dac(uint16_t value) {
  Wire.beginTransmission(DAC_I2C_ADDRESS);
  Wire.write(DAC_WRITE_COMMAND);
  Wire.write(value / 16);
  Wire.write((value % 16) << 4);
  Wire.endTransmission();
}
