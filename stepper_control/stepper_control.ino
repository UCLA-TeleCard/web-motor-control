/*
  Control a bi-polar stepper motor using the SparkFun ProDriver TC78H670FTG
  By: Pete Lewis
  SparkFun Electronics
  Date: July 2nd, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example does a custom setup (1/2 microstep resolution) and turns the motor back and forth.
  
  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/16836

  Hardware Connections:

  ARDUINO --> PRODRIVER
  D8 --> STBY
  D7 --> EN
  D6 --> MODE0
  D5 --> MODE1
  D4 --> MODE2
  D3 --> MODE3
  D2 --> ERR

*/

#include "SparkFun_ProDriver_TC78H670FTG_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_ProDriver
PRODRIVER myProDriver; //Create instance of this object

int leadLimit = 11;
int wheelLimit = 13;
int stepsLead = 0;

int UP = 1;
int DOWN = 0;

void setup() {
  Serial.begin(115200);

  pinMode(leadLimit, INPUT);
  pinMode(wheelLimit, INPUT);

  //***** Configure the ProDriver's Settings *****//
  // Note, we must change settings BEFORE calling the .begin() function.
  // For this example, we will try 1/2 step resolution.
  myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_FULL;

  // The following lines of code are other options you can try out.
  // Comment-out the above settings declaration, and uncomment your desired setting below.
  // myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_FULL;
  // myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_1_2; // 1/2 step
  // myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_1_4; // 1/4 step
  // myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_1_8; // 1/8 step
  // myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_1_16; // 1/16 step
  // myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_1_32; // 1/32 step
  // myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_1_64; // 1/64 step
  // myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_1_128; // 1/128 step

  myProDriver.begin(); // adjust custom settings before calling this


}

// CW = 0
// CCW = 1

void loop() {
  if (Serial.available()) {  // check for incoming serial data
    String command = Serial.readString();  // read command from serial port
    // split command into motor and step components
    String motor = String(command.charAt(0));
    command.remove(0, 1);
    int steps = command.toInt();
    
    // choose the motor, direction, and steps
    if(motor == "L"){
      stepsLead += steps;
      Serial.println(stepsLead);
      myProDriver.step(steps, UP);
    }
    else if (motor == "R"){
      stepsLead -= steps;
      Serial.println(stepsLead);
      myProDriver.step(steps, DOWN);
    }
    // ZEROING PROCESS
    else if (motor == "Z"){
      stepsLead = 0;
      Serial.println(stepsLead);
      // if already zeroed, move up and try again
      if(digitalRead(leadLimit) == HIGH){
        myProDriver.step(200, UP);
        delay(50);
      }
      // keep checking for limit switch while moving down
      while(digitalRead(leadLimit) != HIGH){
        myProDriver.step(1, DOWN);
      }
    }    
  }
}
