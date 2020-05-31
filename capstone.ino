#include <Dac.h>
#include <Encoder.h>
#include <Path.h>
#include <Controller.h>

int i = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  setupController();
  dac.begin();
  setupEncoder();
  path.begin(1.0, micros());
}


void loop() {
  if (i++ < 500){
    Serial.print(y_goal * 1000);
    Serial.write(' ');
    Serial.println(y * 1000);
    delay(1);
  }
}
