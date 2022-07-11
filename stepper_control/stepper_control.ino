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

  This example controls two separate ProDrivers using Serial mode and individual LATCH pins.
  When using Serial Mode, we can share all of the control pins with many ProDrivers,
  But each ProDriver will need it's own unique latch pin.
  Before calling .begin(), we will set the latch pin as needed.
  
  Things to note:
    - Every control pin is shared EXCEPT latch (aka mode1Pin), so we must ensure all latches are low during
      ANY instances call to begin().
    - only full step (aka 1:1) control is supported via serial mode. No microstepping.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/16836

  Hardware Connections:

  ARDUINO --> PRODRIVERs
  D8 --> STBY   PRODRIVER1 and PRODRIVER2
  D7 --> EN     PRODRIVER1 and PRODRIVER2
  D6 --> MODE0  PRODRIVER1 and PRODRIVER2
  D5 --> MODE1  ***PRODRIVER1 only***       (LATCH PIN 1)
  D4 --> MODE2  PRODRIVER1 and PRODRIVER2
  D3 --> MODE3  PRODRIVER1 and PRODRIVER2
  D2 --> ERR    PRODRIVER1 and PRODRIVER2

  ARDUINO --> PRODRIVER2
  D9 --> MODE1 ***PRODRIVER2 only***       (LATCH PIN 2)


*/



#include "SparkFun_ProDriver_TC78H670FTG_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_ProDriver
PRODRIVER wheelDriver; //Create instance of this object
PRODRIVER leadDriver;
#define leadDriverLatchPin 9


int leadLimit = 11;
int wheelLimit = 13;
int stepsLead = 0;

int UP = 1;
int DOWN = 0;
int LEFT = 1;
int RIGHT = 0;

int leadMax = 2700;
int leadMin = 0;

int wheelSlowDown = 10;

bool isZeroed = false;
unsigned long timeout = 10000;

void setup() {
  Serial.begin(115200);

  pinMode(leadLimit, INPUT);
  pinMode(wheelLimit, INPUT);

  //***** Configure the ProDriver's Settings *****//
  // Note, we must change settings BEFORE calling the .begin() function.
  // For this example, we will try 1/2 step resolution.
//  myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_FULL;/

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


  // myProDriver2
  // Note, this must be setup first because of shared lines.
  leadDriver.settings.controlMode = PRODRIVER_MODE_SERIAL;
  leadDriver.settings.mode1Pin = leadDriverLatchPin; // latch pin
  leadDriver.begin(); // calling this first ensure latch pin 2 will be low during other future .begin()s
  leadDriver.setCurrentLimit(950); // 25% current limit
//  leadDriver.setTorque(PRODRIVER_TRQ_25); // 25% torque limit/


  // myProDriver1
  // default latch pin is D5, so no need to change here
  wheelDriver.settings.controlMode = PRODRIVER_MODE_SERIAL;
  wheelDriver.begin();
  wheelDriver.setCurrentLimit(650);


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

    // a "go to location" command for the lead screw
    // checks that stepper is zeroed first
    if (motor == "X" && isZeroed){
      // ensures the position is within the possible range
      steps = constrain(steps, leadMin, leadMax);
      Serial.println(steps);
      int stepsDiff = steps - stepsLead;
      // if positive diff, stepper needs to move up
      if (stepsDiff > 0){
        leadDriver.stepSerial(stepsDiff, UP);
      }
      // if negative, stepper moves down
      else {
        leadDriver.stepSerial(abs(stepsDiff), DOWN);
      }
      stepsLead = steps;
    }

    else if (motor == "U"){
      stepsLead += steps;
      Serial.println(stepsLead);
      leadDriver.stepSerial(steps, UP);
    }
    else if (motor == "D"){
      stepsLead -= steps;
      Serial.println(stepsLead);
      leadDriver.stepSerial(steps, DOWN);
    }
    else if (motor == "L"){
//      stepsLead -= steps;/
      Serial.println(stepsLead);
      for(int i = 0; i <= steps; i++){
        wheelDriver.stepSerial(1, LEFT);
        delay(wheelSlowDown);
      }
    }
    else if (motor == "R"){
//      stepsLead -= steps;/
      Serial.println(stepsLead);
      for(int i = 0; i <= steps; i++){
        wheelDriver.stepSerial(1, RIGHT);
        delay(wheelSlowDown);
      }
    }
    
    // ZEROING PROCESS
    else if (motor == "Z"){
      stepsLead = 0;
      Serial.println(stepsLead);
      // if already zeroed, move up and try again
      if(digitalRead(leadLimit) == HIGH){
        leadDriver.stepSerial(200, UP);
        delay(50);
      }
      unsigned long timeStart = millis();
      // keep checking for limit switch while moving down
      while(digitalRead(leadLimit) != HIGH && millis()-timeStart <= timeout){
        leadDriver.stepSerial(1, DOWN);
      }

      // repeat the process for wheel stepper
      wheelDriver.setCurrentLimit(1023);
      if(digitalRead(wheelLimit) == HIGH){
        wheelDriver.stepSerial(100, RIGHT);
        delay(50);
      }
        timeStart = millis();
      // keep checking for limit switch while moving left
      while(digitalRead(wheelLimit) != HIGH && millis()-timeStart <= timeout/2){
        wheelDriver.stepSerial(1, LEFT);
        delay(wheelSlowDown);
      }
      delay(200);
      wheelDriver.stepSerial(17, LEFT);
      wheelDriver.setCurrentLimit(650);
      isZeroed = true;
    }    
  }
}
