#ifndef Dac_h
#define Dac_h

#include <Wire.h>

// DAC hardware constants
#define SDA_PIN PB7
#define SCL_PIN PB6
#define DAC_I2C_ADDRESS 0x60
#define DAC_WRITE_COMMAND 0x40

// Measured min/max amp values
#define VMIN -9.35
#define VMAX 9.33

// Define Dac class
class Dac {
  public:
    void begin();
    void write(double value);
};

// Setup I2C communication and output 0V
void Dac::begin(){
  Wire.begin();
  write(0);
}

// Write voltage to DAC over I2C
void Dac::write(double value) {

  // Rescale value from +/- volts to 0-4095 integer
  value = 4095.0 * (value - VMIN) / (VMAX - VMIN);

  // Limit output to valid range
  if (value < 0) value = 0;
  if (value > 4095) value = 4095;
  uint16_t dac_value = uint16_t(value);

  // Write to I2C correctly
  Wire.beginTransmission(DAC_I2C_ADDRESS);
  Wire.write(DAC_WRITE_COMMAND);
  Wire.write(dac_value / 16);          // Low byte
  Wire.write((dac_value % 16) << 4);   // High byte
  Wire.endTransmission();
}

Dac dac;

#endif
