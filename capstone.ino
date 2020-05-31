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
double vel;

// Encoder Counter
#define SEL_PIN PB0
#define EO_PIN PB1
#define CLK_PIN PB3
#define RST_PIN PB11
HardwareTimer clkTimer(TIM2);
uint16_t encoderPos = 0;

// DAC
#define SDA_PIN PB7
#define SCL_PIN PB6
#define DAC_I2C_ADDRESS 0x60
#define DAC_WRITE_COMMAND 0x40

// Curve Calculation
#define T1_POW 2
#define T2_POW 3
#define T3_POW 4
const double P_T1[] = {1.5905365e-04,  -1.2742564e-02,   8.1071910e-02};
const double P_T2[] = {7.6933361e-05,   2.0894041e-03,  -2.4460736e-03,   2.0637769e-01};
const double P_T3[] = {7.6417970e-03,  -5.4250209e-02,   1.8030993e-01,  -4.2988315e-01,   6.9129266e-01};
double t1, t2, t3;

// Controller
HardwareTimer sampleTimer(TIM3); // TODO: Check if TIM3 works for controller timer
double y_goal;
double y;
double controllerEffort;
int myFilter_ns = 1; // No. of sections
struct biquad {
  double b0; double b1; double b2;  // numerator
  double a0; double a1; double a2;  // denominator
  double x0; double x1; double x2;  // input
  double y1; double y2;};           // output
static struct biquad controllerBiquad[] = {{
  4836.780688, -4549.787648, 0.000000, 
  1.000000, -0.711621, 0.000000, 
  0, 0, 0, 0, 0
}};

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
float goalPositions[SAMPLE_SIZE];
float actualPositions[SAMPLE_SIZE];
unsigned int times[SAMPLE_SIZE];
unsigned int sampleIndex = 0;


void setup() {
  setupDac();                 // Start I2C for DAC
  setupSerial();              // Setup serial
  
  setupController();          // Configure controller timer
  setupEncoder();             // Configure encoder counter
//  setupPhotosensors();        // Configure interrups for photosensors
  interrupt2();
//  delay(1000);
//  for (int i = 0; i < SAMPLE_SIZE; i++){
//    Serial.print(goalPositions[i], 5);
////    Serial.write(' ');
////    Serial.print(times[i]);
//    Serial.write(' ');
//    Serial.println(actualPositions[i]);
//  }
}


void loop() {
  Serial.print(y_goal * 1000.0);
  Serial.write(' ');
  Serial.print(y * 1000.0);
  Serial.write(' ');
  Serial.println(controllerEffort * 100);
  delay(30);
}


void setupSerial(){
  Serial.begin(BAUD);
  while (!Serial);
}

void setupController(){
  sampleTimer.pause(); 
  sampleTimer.setMode(1, TIMER_OUTPUT_COMPARE);
  sampleTimer.setPrescaleFactor(36000);  // Divide to 2kHz
  sampleTimer.setOverflow(2);            // Divide to 1kHz
  sampleTimer.setCaptureCompare(1, 1);
  sampleTimer.attachInterrupt(1, callback);
  sampleTimer.refresh();
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
  delayMicroseconds(4);
  uint16_t hb = GPIOA->IDR & 0x00FF;
  digitalWrite(SEL_PIN, LOW);
  delayMicroseconds(4);
  uint16_t lb = GPIOA->IDR & 0x00FF;
  digitalWrite(EO_PIN, LOW);
  delayMicroseconds(4);
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
  writeDac(2048);
}

void writeDac(uint16_t value) {
  Wire.beginTransmission(DAC_I2C_ADDRESS);
  Wire.write(DAC_WRITE_COMMAND);
  Wire.write(value / 16);
  Wire.write((value % 16) << 4);
  Wire.endTransmission();
}

void callback(HardwareTimer *timer){
  
  unsigned int encCount = readEncoder();
  y = encCount / 10185.91636;
  y_goal = getTargetPos(0.0787, 0.2530, 0.4069, 1.5);
  y_goal += 0.5;
//  y_goal = 0.5;
  double err = y_goal - y;
  controllerEffort = getControllerEffort(err) / 10;
  if (controllerEffort > 10.0) controllerEffort = 10.0;
  if (controllerEffort < -10.0) controllerEffort = -10.0;
  writeDac(map(controllerEffort, -10, 10, 0, 4095));

  if (sampleIndex < SAMPLE_SIZE){
    goalPositions[sampleIndex] = y_goal * 1000;
    actualPositions[sampleIndex] = y * 1000;
    times[sampleIndex] = micros() / 1000;
    sampleIndex++;
  }
}

void setupPhotosensors(){
  //set interrupt pins to inputs
  pinMode(PS_PIN1, INPUT); //sets PhotoSesnor 1 to input
  attachInterrupt(digitalPinToInterrupt(PS_PIN1), interrupt1, RISING); //Sets up interrupt on change in PhotoSensor1Pin to run CalcV0
  
  pinMode(PS_PIN2, INPUT); //sets PhotoSesnor 2 to input
  attachInterrupt(digitalPinToInterrupt(PS_PIN2), interrupt2, RISING); //Sets up interrupt on change in PhotoSensor2Pin to run CalcV0
}

void interrupt1(){
  psTime1 = micros();
}

void interrupt2(){
  psTime2 = micros();
  sampleTimer.resume();
}

double calcVelocity(){
  vel = PS_DISTANCE / (psTime2 - psTime1);
  return vel;
}

void getTimes(double v0){
  t1 = 0; t2 = 0; t3 = 0;
  for (int i = 0; i <= T1_POW; i++){
    t1 += P_T1[i] * pow(v0, T1_POW - i);
  }
  for (int i = 0; i <= T2_POW; i++){
    t2 += P_T2[i] * pow(v0, T2_POW - i);
  }
  for (int i = 0; i <= T2_POW; i++){
    t3 += P_T3[i] * pow(v0, T2_POW - i);
  }
}

double getTargetPos(double t1,double t2, double t3, double v0){
  double fact = -1.0667 * v0 + 4;
  double tau2 = TAU * fact;
  double aConst = -(v0 + G *t2) / (t2 - t3); // original : t2-t3
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

double cascade( double xin,        // input
        struct biquad *fa,         // biquad array
        int ns){                   // no. segments
  struct biquad* f = fa;
  f->x0 = xin;
  double y0;
  int i;
  for(i = 0; i < ns; i++){
    // passes previous values
    if ( i > 0){
      f->x0 = y0;
    }
    // calculate y0
    y0 = (f->b0*f->x0 + f->b1*f->x1 + f->b2*f->x2 - f->a1*f->y1 - f->a2*f->y2)/f->a0;
    //update x and y
    f->x2 = f->x1; f->x1 = f->x0;
    f->y2 = f->y1; f->y1 = y0;
    f++;  // increment f
  }
  return y0;
};

double getControllerEffort(double error){
  return cascade(error, controllerBiquad, 1);
}
