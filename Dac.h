#ifndef Dac_h
#define Dac_h

#include <Wire.h>

#define SDA_PIN PB7
#define SCL_PIN PB6
#define DAC_I2C_ADDRESS 0x60
#define DAC_WRITE_COMMAND 0x40

class Dac {
  public:
    void begin();
    void write(uint16_t value);
};

void Dac::begin(){
  Wire.begin();
  write(2048);
}

void Dac::write(uint16_t value) {
  Wire.beginTransmission(DAC_I2C_ADDRESS);
  Wire.write(DAC_WRITE_COMMAND);
  Wire.write(value / 16);
  Wire.write((value % 16) << 4);
  Wire.endTransmission();
}

Dac dac;

#endif
