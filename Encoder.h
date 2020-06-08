#ifndef Encoder_h
#define Encoder_h

#define SEL_PIN PB0
#define EO_PIN PB1
#define CLK_PIN PB3
#define RST_PIN PB11

HardwareTimer clkTimer(TIM2); // Clock for HCTL-2000 IC
uint16_t encoderPos = 0x1000;

// Configure timer, pins, io bank
void setupEncoder(){

  // Hardware pins to HCTL-2000, inverted  
  pinMode(SEL_PIN, OUTPUT);
  pinMode(EO_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
  digitalWrite(SEL_PIN, LOW);
  digitalWrite(EO_PIN, LOW);

  // Configure GPIOA bank for input
  GPIOA->CRL = 0x88888888;

  // Configure timer
  clkTimer.pause();
  clkTimer.setPrescaleFactor(10);    // Downsample to 7.2MHz
  clkTimer.setOverflow(6);           // Downsample to 1.2MHz
  clkTimer.setCaptureCompare(2, 3);  // Compare for pin output
  clkTimer.setMode(2, TIMER_OUTPUT_COMPARE_PWM1, CLK_PIN);
  clkTimer.refresh();
  clkTimer.resume();
}

uint16_t readEncoder(){

  // Read high byte
  digitalWrite(EO_PIN, HIGH);
  digitalWrite(SEL_PIN, HIGH);
  delayMicroseconds(4);
  uint16_t hb = GPIOA->IDR & 0x00FF;

  // Read low byte
  digitalWrite(SEL_PIN, LOW);
  delayMicroseconds(4);
  uint16_t lb = GPIOA->IDR & 0x00FF;

  // Resume counting
  digitalWrite(EO_PIN, LOW);
  delayMicroseconds(4);

  // Combine high and low bytes to get position
  uint16_t thisPos = (hb << 8) | lb;

  // Handle overflow of 12 bit value
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

#endif
