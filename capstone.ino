#include <Dac.h>
#include <Encoder.h>
#include <Path.h>
#include <Controller.h>
#include <Photo.h>

String s = "";
bool plot = true;
enum PlotMode {
  RUNNING,
  RECORD,
} plotMode = RUNNING;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  dac.begin();
  setupEncoder();
  setupController();
//  setupPhotointerrupters();
}


void loop() {

  switch (plotMode){
    case RUNNING:
      Serial.print(y_goal * 1000);
      Serial.print(' ');
      Serial.print(y * 1000);
      Serial.write(' ');
      Serial.print(controllerEffort * 100);
      Serial.println();
      delay(20);
      break;
    case RECORD:
      if (idx == FRAMES){
        Serial.println("y_goal\ty\tcontrollerEffort");
        for (int i = 0; i < FRAMES; i++){
          Serial.print(y_goal_arr[i], 5);
          Serial.write('\t');
          Serial.print(y_arr[i], 5);
          Serial.write('\t');
          Serial.print(controllerEffort_arr[i], 5);
          Serial.println();
        }
        idx++;
      }
      break;
    
  }

  if (Serial.peek() == ';'){
    Serial.read();
    if (s.length() >= 2 && s[0] == 'M'){
      switch (s[1]){
        case '1':
          plotMode = RUNNING;
          break;
        case '2':
          plotMode = RECORD;
          if (s.length() > 2){
            y_goal = double(s.substring(2).toInt()) / 1000.0;
            idx = 0;
          }
          break;
        case '3':
          currentMode = WAIT;
          break;
        case '4':
          currentMode = RUN;
          break;
      }
    } else if (s.length() >= 2 && s[0] == 'V'){
      limMaxEffort = s.substring(1).toFloat();
    } else if (s.length() >= 2 && s[0] == 'T') {
      path.begin(s.substring(1).toFloat());
      idx = 0;
    } else {
      y_goal = double(s.toInt()) / 1000.0;
      idx = 0;
    }
    s = "";
  } else if (Serial.peek() != -1) {
    s.concat(char(Serial.read()));
  }
}
