/*

SERIAL INTERFACE:
Many variable can be adjusted on the fly via uart serail interface.
This is done by entering commands. Commands are ASCII strings that
end with ';'.

M{MODE}; - Set Mode
  M1; - Active by default. Continuous plotting mode. Will output
        current y_goal, y, and controller effor ever 20 ms. Units
        are in [mm] and [10x mV] for scale.
  
  M2; - Logging mode. 500 data samples of y_goal, y and controller
        effort are recorded. This happens every time a position is
        inputted if M3 is active or evertime a path is triggered
        if M4 is active.

  M3; - Active by default. User control. Controller will move to any
        user given position. EX '100;' will step to 100mm. Path
        triggers will be ignored when M3 is active.

  M4; - Run mode. Controller will follow trajectories given by path.
        Any user given positions will be ignored. Paths can be
        trigger via photointerrupters or via T commands.

V{MAX}; - Default 2V. Sets the max controller effort in volts.
          Ex 'V10.0;' limits effort to +/- 10V.

T{V0}; - Trigger. Triggers a path for a given initial velocity.
         Ex. 'T1.5;' will simulate an egg with an initial velocity
         of 1.5 m/s. Will be ignored when M3 is active.

K{COEFF}; - Gain Adjust. Default 0.1. Sets a coefficient to multiply
            controller effort by effectively changing the controller
            stiffness. Ex. 'K1.0;' will use the designed Kc value.

P{DIST}; - Photosensor Distance. Default 44mm. Sets the distance
           between photosensors in [mm].

O{DIST}; - Offset Distance. Default 0.0. Distance to offset zero
           position by. Used to fine tune the height that the egg is
           caught at. Ex 'O-20.0' lowers the zero point 20mm.

S{COEFF}; - Softening Factor. Similar to the K command but only effects
            Kc after t2 where the egg is in the cariage. Used to add
            complience to the controller. Ex. 'S0.05' divides Kc by 20
            after egg is caught. 

*/

#include <Dac.h>
#include <Encoder.h>
#include <Path.h>
#include <Controller.h>
#include <Photo.h>

String s = "";  // Used to build user commands
enum PlotMode {
  RUNNING,
  RECORD,
  VELOCITY,
} plotMode = RUNNING;

// Runs once when booted up
void setup() {
  dac.begin();
  Serial.begin(9600);
  while (!Serial);       // Wait for user to open serial connection
  setupEncoder();
  setupController();
  setupPhotointerrupters();
}

// Runs continuously. Manages user interface and plotting/logging
void loop() {

  // Implementation for plotting modes  
  switch (plotMode){

    // Continuously plot y_goal, y in mm and controllerEffort in 0.1 x mV
    case RUNNING:
      Serial.print(y_goal * 1000);
      Serial.print(' ');
      Serial.print(y * 1000);
      Serial.write(' ');
      Serial.print(controllerEffort * 100);
      Serial.println();
      delay(20);
      break;

    // Record 500 actual controller samples and report at end.
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

    // Plot the current V0 and photointerrupter states
    case VELOCITY:
      Serial.print(v0 * 100, 5);
      Serial.write(' ');
      Serial.print(200 * digitalRead(PS_PIN1));
      Serial.write(' ');
      Serial.print(200 * digitalRead(PS_PIN2));
      Serial.println();
      delay(20);
      break;
  }

  if (Serial.peek() == ';'){  // Chech if cammand is finished
    Serial.read();

    // Mode command    
    if (s.length() >= 2 && s[0] == 'M'){
      switch (s[1]){
        case '1':  // M1; Continuous plotting
          plotMode = RUNNING;
          break;
        case '2':  // M2; Record 500 samples
          plotMode = RECORD;
          if (s.length() > 2){
            y_goal = double(s.substring(2).toInt()) / 1000.0;
            idx = 0;
          }
          break;
        case '3':  // M3; User control position. Not path defined
          currentMode = WAIT;
          break;
        case '4':  // M4; Path controlled position. Is triggered by photointerrupters
          currentMode = RUN;
          break;
        case '5':  // M5; Plot V0 and photointerrupter states
          plotMode = VELOCITY;
          break;
      }
    } else if (s.length() >= 2 && s[0] == 'V'){      // V{V_MAX}; Set the max controller effort in volts
      limMaxEffort = s.substring(1).toFloat();
    } else if (s.length() >= 2 && s[0] == 'T') {     // T{V0}; Trigger a catch at given velocity in m/s
      path.begin(s.substring(1).toFloat());
      idx = 0;
    } else if (s.length() >= 2 && s[0] == 'K') {     // K{COEFF}; Set the K adjustment. 'K1;' is original value
      K_adj = s.substring(1).toFloat();
    } else if (s.length() >= 2 && s[0] == 'P') {     // P{DIST}; Set the distance between sensors in mm
      ps_distance = s.substring(1).toFloat() / 1000;
    } else if (s.length() >= 2 && s[0] == 'O') {     // O{DIST}; Set the zero offset in mm.
      zeroOffset = s.substring(1).toFloat() / 1000;
    } else if (s.length() >= 2 && s[0] == 'S') {     // S{COEFF}; Set the softening value. 'S1;' leaves K unchanged
      soften_adj = s.substring(1).toFloat();
    } else {                                         // Otherwise go to user set location
      y_goal = double(s.toInt()) / 1000.0;
      idx = 0;
    }
    s = "";  // Reset command buffer
  } else if (Serial.peek() != -1) {  // Add character to buffer
    s.concat(char(Serial.read()));
  }
}
