#ifndef Dac_h
#define Dac_h

#include <Wire.h>

#define SDA_PIN PB7
#define SCL_PIN PB6
#define DAC_I2C_ADDRESS 0x60
#define DAC_WRITE_COMMAND 0x40

#define VMIN -9.35
#define VMAX 9.33

class Dac {
  public:
    void begin();
    void write(double value);
};

void Dac::begin(){
  Wire.begin();
  write(2048);
}

void Dac::write(double value) {
  value = 4095.0 * (value - VMIN) / (VMAX - VMIN);
  if (value < 0) value = 0;
  if (value > 4095) value = 4095;
  uint16_t dac_value = uint16_t(value);
  Wire.beginTransmission(DAC_I2C_ADDRESS);
  Wire.write(DAC_WRITE_COMMAND);
  Wire.write(dac_value / 16);
  Wire.write((dac_value % 16) << 4);
  Wire.endTransmission();
}

Dac dac;

#endif
