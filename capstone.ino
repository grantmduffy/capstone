#include <Wire.h>

// Serial
#define BAUD 115200

// Photosensors
#define PS_PIN1 1         // pin which photosensor 1 is connected to. Left blank for now TODO: Correct pin
#define PS_PIN2 2         // pin which photosensor 2 is connected to. Left blank for now TODO: Correct pin
#define PS_DISTANCE 1000  // distance between the photosensors, unit in mm. TODO: Correct the distance
unsigned long psTime1;  // time first photsensor is triggered
unsigned long psTime2;  // initial velocity
unsigned long t1, t2, t3;
double vel;

// Encoder Counter
#define SEL_PIN PB0
#define EO_PIN PB1
#define CLK_PIN PB3
#define RST_PIN PB11
HardwareTimer clkTimer(TIM2);
uint16_t encoderPos;

// DAC
#define SDA_PIN PB7
#define SCL_PIN PB6
#define DAC_I2C_ADDRESS 0x60
#define DAC_WRITE_COMMAND 0x40

// Controller
#define LOOKUP_TABLE_SIZE 256
HardwareTimer sampleTimer(TIM3); // TODO: Check if TIM3 works for controller timer
unsigned int lookupTable[LOOKUP_TABLE_SIZE];


void setup() {
  setupSerial();         // Set serial
//  setupDac();            // Start I2C for DAC
//  setupController();     // Configure controller timer
  setupEncoder();        // Configure encoder counter
//  setupPhotosensors();   // Configure interrups for photosensors
}


void loop() {
  Serial.println(readEncoder());
  delay(100);
//  while (psTime1 == 0 && psTime2 == 0);
//  double initial_velocity  = calc_velocity();
//  lookup_times();
//
//  psTime1 = 0;
//  psTime2 = 0;
}


void setupSerial(){
  Serial.begin(BAUD);
  while (!Serial);
}

void setupController(){
  sampleTimer.pause(); 
  sampleTimer.setMode(1, TIMER_OUTPUT_COMPARE);
  sampleTimer.setPrescaleFactor(65536);  // TODO: Configure for sample rate
  sampleTimer.setOverflow(1099);         // TODO: Configure for sample rate
  sampleTimer.setCaptureCompare(1, 1);
  sampleTimer.attachInterrupt(1, callback);
  sampleTimer.refresh();
  sampleTimer.resume();
}

void setupEncoder(){
  pinMode(SEL_PIN, OUTPUT);
  pinMode(EO_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
  digitalWrite(SEL_PIN, LOW);
  digitalWrite(EO_PIN, LOW);
  GPIOA->CRL = 0x88888888;
  clkTimer.pause();
  clkTimer.setPrescaleFactor(10);
  clkTimer.setOverflow(6);
  clkTimer.setCaptureCompare(2, 3);
  clkTimer.setMode(2, TIMER_OUTPUT_COMPARE_PWM1, CLK_PIN);
  clkTimer.refresh();
  clkTimer.resume();
}

uint16_t readEncoder(){
  digitalWrite(EO_PIN, HIGH);
  digitalWrite(SEL_PIN, HIGH);
  delayMicroseconds(2);
  uint16_t hb = GPIOA->IDR & 0x00FF;
  digitalWrite(SEL_PIN, LOW);
  delayMicroseconds(2);
  uint16_t lb = GPIOA->IDR & 0x00FF;
  digitalWrite(EO_PIN, LOW);
  delayMicroseconds(2);
  uint16_t thisPos = (hb << 8) | lb;
  if ((thisPos < 1024) && ((encoderPos & 0x0FFF) > 3072)){
    thisPos |= (encoderPos & 0xF000) + (1 << 12);
    encoderPos = thisPos;
  } else if ((thisPos > 3072) && ((encoderPos & 0x0FFF) < 1024)){
    thisPos |= (encoderPos & 0xF000) - (1 << 12);
    encoderPos = thisPos;
  } else {
    encoderPos = thisPos | (encoderPos & 0xF000);
  }
  return encoderPos;
}

void setupDac(){
  Wire.begin();
}

void writeDac(uint16_t value) {
  Wire.beginTransmission(DAC_I2C_ADDRESS);
  Wire.write(DAC_WRITE_COMMAND);
  Wire.write(value / 16);
  Wire.write((value % 16) << 4);
  Wire.endTransmission();
}

void callback(HardwareTimer *timer){
  // TODO: ISR for controller
  Serial.println("Interrupt!");
}

void setupPhotosensors(){
  //set interrupt pins to inputs
  pinMode(PS_PIN1, INPUT); //sets PhotoSesnor 1 to input
  attachInterrupt(digitalPinToInterrupt(PS_PIN1), interrupt_1, RISING); //Sets up interrupt on change in PhotoSensor1Pin to run CalcV0
  
  pinMode(PS_PIN2, INPUT); //sets PhotoSesnor 2 to input
  attachInterrupt(digitalPinToInterrupt(PS_PIN2), interrupt_2, RISING); //Sets up interrupt on change in PhotoSensor2Pin to run CalcV0
}

void interrupt_1(){
  psTime1 = millis();
}

void interrupt_2(){
  psTime2 = millis();
}

double calc_velocity(){
  vel = PS_DISTANCE / (psTime2 - psTime1);
  return vel;
}

void lookup_times(double initial_velocity){
  for(int i = 0; i < LOOKUP_TABLE_SIZE; i++){
    if(lookupTable[i, 0] == initial_velocity){
      t1 = lookupTable[i, 1];
      t2 = lookupTable[i, 2];
      t3 = lookupTable[i, 3];
    } else if(lookupTable[i + 1, 0] > initial_velocity && lookupTable[i, 0] < initial_velocity){
      t1 = (lookupTable[i + 1, 1] - lookupTable[i, 1]) / (lookupTable[i + 1, 0] - lookupTable[i, 0]) * (initial_velocity - lookupTable[i, 0]) + lookupTable[i, 1];
      t2 = (lookupTable[i + 1, 2] - lookupTable[i, 2]) / (lookupTable[i + 1, 0] - lookupTable[i, 0]) * (initial_velocity - lookupTable[i, 0]) + lookupTable[i, 2];
      t3 = (lookupTable[i + 1, 3] - lookupTable[i, 3]) / (lookupTable[i + 1, 0] - lookupTable[i, 0]) * (initial_velocity - lookupTable[i, 0]) + lookupTable[i, 3];
    }
  }
}

unsigned int get_target_position(){
  return 0;
}
