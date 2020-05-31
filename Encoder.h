#ifndef Encoder_h
#define Encoder_h

#define SEL_PIN PB0
#define EO_PIN PB1
#define CLK_PIN PB3
#define RST_PIN PB11

HardwareTimer clkTimer(TIM2);
uint16_t encoderPos = 0;

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

#endif
