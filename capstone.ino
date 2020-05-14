#include <Wire.h>

// Designate interrupt pins
#define PS_PIN1 = 1; //pin which photosensor 1 is connected too. Left blank for now TODO: Correct pin
#define PS_PIN2 = 2; //pin which photosensor 2 is connected too. Left blank for now TODO: Correct pin

// Bunch of variables for the photosensor interrupt.  Global for now but will update to pass them between functions
unsigned long ps_time_1; //time first photsensor is triggered
unsigned long ps_time_2; //initial velocity
unsigned long t1, t2;

// Parameters for I2C
#define DAC_I2C_ADDRESS 0x60
#define DAC_WRITE_COMMAND 0x40

void setup() {
  Serial.begin(115200);  // Set client serial freq to 115200
  Wire.begin();          // Start I2C for DAC

  TIM2->CR1 |= 0x00;
  
  //set interrupt pins to inputs
//  pinMode(PhotoSensor1Pin, INPUT); //sets PhotoSesnor 1 to input
//  attachInterrupt(digitalPinToInterrupt(PhotoSensor1Pin), CalcV0, RISING); //Sets up interrupt on change in PhotoSensor1Pin to run CalcV0
//  
//  pinMode(PhotoSensor2Pin, INPUT); //sets PhotoSesnor 2 to input
//  attachInterrupt(digitalPinToInterrupt(PhotoSensor2Pin), CalcV0, RISING); //Sets up interrupt on change in PhotoSensor2Pin to run CalcV0
}

void loop() {

  for (int val = 0; val < 4096; val+=10){
    write_dac(val);
    delay(10);
    Serial.println(analogRead(A0));
  }

  
//  while (ps_time_1 == 0 && ps_time_2 == 0){}
//  calc_velocity();
//  lookup_times();

//  (controller loop)

//  t1 = 0;
//  t2 = 0;
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

void write_dac(uint16_t value) {
  Wire.beginTransmission(DAC_I2C_ADDRESS);
  Wire.write(DAC_WRITE_COMMAND);
  Wire.write(value / 16);
  Wire.write((value % 16) << 4);
  Wire.endTransmission();
}
