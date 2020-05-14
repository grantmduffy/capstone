#include <Wire.h>

// Designate interrupt pins
#define PS_PIN1 1         // pin which photosensor 1 is connected too. Left blank for now TODO: Correct pin
#define PS_PIN2 2         // pin which photosensor 2 is connected too. Left blank for now TODO: Correct pin
#define PS_DISTANCE 1000  // distance between the photosensors, unit in mm. TODO: Correct the distance
#define LOOKUP_TABLE_SIZE 256

// Bunch of variables for the photosensor interrupt.  Global for now but will update to pass them between functions
unsigned long ps_time_1;  // time first photsensor is triggered
unsigned long ps_time_2;  // initial velocity
unsigned long t1, t2, t3;
double vel;
unsigned int lookup_table[LOOKUP_TABLE_SIZE];

// Parameters for I2C
#define DAC_I2C_ADDRESS 0x60
#define DAC_WRITE_COMMAND 0x40

// Timer
HardwareTimer sample_timer(TIM2);

void setup() {
  Serial.begin(115200);  // Set client serial freq to 115200
  Wire.begin();          // Start I2C for DAC
  setup_timer();         // Configure timer

  //set interrupt pins to inputs
  pinMode(PS_PIN1, INPUT); //sets PhotoSesnor 1 to input
  attachInterrupt(digitalPinToInterrupt(PS_PIN1), interrupt_1, RISING); //Sets up interrupt on change in PhotoSensor1Pin to run CalcV0
  
  pinMode(PS_PIN2, INPUT); //sets PhotoSesnor 2 to input
  attachInterrupt(digitalPinToInterrupt(PS_PIN2), interrupt_2, RISING); //Sets up interrupt on change in PhotoSensor2Pin to run CalcV0
}

void loop() {
  while (ps_time_1 == 0 && ps_time_2 == 0);
  double initial_velocity  = calc_velocity();
  lookup_times();

  ps_time_1 = 0;
  ps_time_2 = 0;
}

void setup_timer(){
  sample_timer.pause();
  sample_timer.setMode(1, TIMER_OUTPUT_COMPARE);
  sample_timer.setPrescaleFactor(65536);  // TODO: Configure for sample rate
  sample_timer.setOverflow(1099);         // TODO: Configure for sample rate
  sample_timer.setCaptureCompare(1, 1);
  sample_timer.attachInterrupt(1, callback);
  sample_timer.refresh();
  sample_timer.resume();
}

void callback(HardwareTimer *timer){
  // TODO: ISR for controller
  Serial.println("Interrupt!");
}

void interrupt_1(){
  ps_time_1 = millis();
}

void interrupt_2(){
  ps_time_2 = millis();
}

double calc_velocity(){
  vel = PS_DISTANCE / (ps_time_2 - ps_time_1);
  return vel;
}

void lookup_times(double initial_velocity){
  for(int i = 0; i < LOOKUP_TABLE_SIZE; i++){
    if(lookup_table[i, 0] == initial_velocity){
      t1 = lookup_table[i, 1];
      t2 = lookup_table[i, 2];
      t3 = lookup_table[i, 3];
    } else if(lookup_table[i + 1, 0] > initial_velocity && lookup_table[i, 0] < initial_velocity){
      t1 = (lookup_table[i + 1, 1] - lookup_table[i, 1]) / (lookup_table[i + 1, 0] - lookup_table[i, 0]) * (initial_velocity - lookup_table[i, 0]) + lookup_table[i, 1];
      t2 = (lookup_table[i + 1, 2] - lookup_table[i, 2]) / (lookup_table[i + 1, 0] - lookup_table[i, 0]) * (initial_velocity - lookup_table[i, 0]) + lookup_table[i, 2];
      t3 = (lookup_table[i + 1, 3] - lookup_table[i, 3]) / (lookup_table[i + 1, 0] - lookup_table[i, 0]) * (initial_velocity - lookup_table[i, 0]) + lookup_table[i, 3];
    }
  }
}

unsigned int get_target_position(){
  return 0;
}

void write_dac(uint16_t value) {
  Wire.beginTransmission(DAC_I2C_ADDRESS);
  Wire.write(DAC_WRITE_COMMAND);
  Wire.write(value / 16);
  Wire.write((value % 16) << 4);
  Wire.endTransmission();
}
