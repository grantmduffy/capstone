#include <Wire.h>
#include <math.h>

// Serial
#define BAUD 115200

// Photosensors
#define PS_PIN1 1         // pin which photosensor 1 is connected to. Left blank for now TODO: Correct pin
#define PS_PIN2 2         // pin which photosensor 2 is connected to. Left blank for now TODO: Correct pin
#define PS_DISTANCE 1000  // distance between the photosensors, unit in mm. TODO: Correct the distance
unsigned long psTime1;    // time first photsensor is triggered
unsigned long psTime2;    // initial velocity
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
double goalPos;

// Design Constants
#define R 0.032               // radius [m]
#define N 1                   // gear ratio
#define W_FREE 401.076662108  // free velocity
#define T_STALL 0.499         // stall torque [N-m]
#define J_EQ 0.000052;        // equivelent inertia
#define G 9.81                // gravitational constant
#define TAU 0.1287            // time constant

// Logging
#define SAMPLE_SIZE 500
double positions[SAMPLE_SIZE];


void setup() {
  setupSerial();              // Setup serial
  setupDac();                 // Start I2C for DAC
//  setupController();          // Configure controller timer
  setupEncoder();             // Configure encoder counter
//  setupPhotosensors();        // Configure interrups for photosensors
  pinMode(PA10, INPUT);
}


void loop() {
//  Test Encoder
//  delay(100);
//  Serial.println(readEncoder());
//  writeDac(encoderPos / 16);

//  while (psTime1 == 0 && psTime2 == 0);
//  double initialVelocity  = calcVelocity();
//  lookupTimes();
//
//  psTime1 = 0;
//  psTime2 = 0;

//  Test goal position function
  psTime2 = millis();
  for (int i = 0; i < SAMPLE_SIZE; i++){
    positions[i] = getTargetPos(0.087169, 0.326034, 0.496697, .5);
    delay(2);
  }
  for (int i = 0; i < SAMPLE_SIZE; i++){
    Serial.println(positions[i], 5);
  }
  while (true);

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
  attachInterrupt(digitalPinToInterrupt(PS_PIN1), interrupt1, RISING); //Sets up interrupt on change in PhotoSensor1Pin to run CalcV0
  
  pinMode(PS_PIN2, INPUT); //sets PhotoSesnor 2 to input
  attachInterrupt(digitalPinToInterrupt(PS_PIN2), interrupt2, RISING); //Sets up interrupt on change in PhotoSensor2Pin to run CalcV0
}

void interrupt1(){
  psTime1 = millis();
}

void interrupt2(){
  psTime2 = millis();
}

double calcVelocity(){
  vel = PS_DISTANCE / (psTime2 - psTime1);
  return vel;
}

void lookupTimes(double initialVelocity){
  for(int i = 0; i < LOOKUP_TABLE_SIZE; i++){
    if(lookupTable[i, 0] == initialVelocity){
      t1 = lookupTable[i, 1];
      t2 = lookupTable[i, 2];
      t3 = lookupTable[i, 3];
    } else if(lookupTable[i + 1, 0] > initialVelocity && lookupTable[i, 0] < initialVelocity){
      t1 = (lookupTable[i + 1, 1] - lookupTable[i, 1]) / (lookupTable[i + 1, 0] - lookupTable[i, 0]) * (initialVelocity - lookupTable[i, 0]) + lookupTable[i, 1];
      t2 = (lookupTable[i + 1, 2] - lookupTable[i, 2]) / (lookupTable[i + 1, 0] - lookupTable[i, 0]) * (initialVelocity - lookupTable[i, 0]) + lookupTable[i, 2];
      t3 = (lookupTable[i + 1, 3] - lookupTable[i, 3]) / (lookupTable[i + 1, 0] - lookupTable[i, 0]) * (initialVelocity - lookupTable[i, 0]) + lookupTable[i, 3];
    }
  }
}

double getTargetPos(double t1,double t2,double t3, double V0){
  double pos;
  double fact = -1.0667 * V0 + 4;
  double tau2 = TAU * fact;
  double aConst = -(V0 + G *t2) / (t2 - t3); // original : t2-t3
  double currTime = ((double) (millis() - psTime2) )/ 1000;
  if (currTime < t1){
    pos = (R / N) * W_FREE * (currTime + TAU * exp(-currTime /TAU) - TAU);
  } else if(currTime < (2 * t1)){
    pos = 2 * ((R / N) * W_FREE * (t1 + TAU * exp(-t1 / TAU) - TAU)) - ((R / N) * W_FREE * (((2 * t1) - currTime) + TAU * exp(-((2 * t1) - currTime) / TAU) - TAU));
  } else if(currTime < t2){
    pos = 2 * ((R / N) * W_FREE * (t1 + TAU * exp(-t1 / TAU) - TAU)) - ((R / N) * W_FREE * ((currTime - 2 * t1) + tau2 * exp(-(currTime - 2 * t1) / tau2) - tau2)); //F2
  } else if (currTime < t3){
    pos = .5 * aConst * (currTime - t3) * (currTime - t3);
  } else{
    pos = 0;
  }
  return pos;
}
