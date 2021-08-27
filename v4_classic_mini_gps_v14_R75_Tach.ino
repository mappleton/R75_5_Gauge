

/*
 * Classic Mini GPS Speedometer
 * Luke Hurst Nov 2019
 * http://retromini.co.uk
 * 
 * Change Log
 * V1 - Basic functionality to allow selection between GPS module or pulse signal
 * V2 - Moved pulses per mile calibration to startup
 * V3 - Removed interrupt save function
 * V4 - 09/11/2019 - Changed to EEPROMWearLevel Library
 * V5 - 11/11/2019 - Added Speed Buffer code
 * V6 - 15/11/2019 - Corrected speed range to match dial. Added if statement to zero speed if below 10mph due to dial starting at 10mph
 * V7 - 06/12/2019 - Added update period to stepper code
 * V8 - 06/12/2019 - Changed motor update function to updateBlocking
 * V9 - 06/12/2019 - Changed motor set position code to ignore speeds beloew 10mph
 * V10 - 06/12/2019 - Added speedo sampling and averaging code
 * V11 - 07/12/2019 - Corrected float calculations to fix speed and step results
 * V12 - 09/12/2019 - Changed motor update from updateBlocking to update. Added code to write PulsesPerMile to memory
 * V13 - 13/12/2019 - Changed odometer code to use single float to store distance travelled. Removed update odomoter function.
 * V14 - 14/12/2019 - Added code to reset odometer to 0 after 99,999 miles to avoid overspill on screen.
 */

//General libraries required
#include <Arduino.h>
#include <Wire.h>
#include <FreqCount.h> // D5 automatically used as input pin.



//Define stepper motor
#include <SwitecX25.h>
const float StepsPerDegree = 3.00;  // Number of steps to move one degree
const unsigned int MaxMotorRotation = 225; // 315 max degrees of movement
const unsigned int MaxMotorSteps = MaxMotorRotation * StepsPerDegree; // Maximum number of steps motor can move
const float GaugeDegreesPerRPM = 10.8; // 90 degrees equals 50mph on the dial. 90/50 provides degrees per mph.
const float SpeedBuffer = 1.03; // Adds 3% to GPS speed to ensure below speed limit
float motorStep = 0.00; // Number of steps to move motor
float PulsesPerRev = 0.5; // Number of pulses per revolution produced by input signal
float ConvertRPM = 0.00; // Used in frequency to RPM conversion
unsigned int PulseCount; // Used to store pulse count captured by FreqCount
float revolutionsPerMinute = 0.00; // Variable to store current speed. 
const int updateInterval = 10;  // 50 milliseconds speedo needle position update rate
unsigned long previousMillis = 0;   // last time we updated the tach
unsigned int dialCorrection = 0; // Dial reading starts at 10mph which = 56 stepper motor steps. This number is subtracted from output to provide accurate indicated speed
SwitecX25 Motor(MaxMotorSteps, 6,7,8,9); // Declare library to be used, call it Motor and define digital pins connected to stepper coils


//Variables used throughout code
#define potmeterPin A1 // Used to calibrate pulses per mile during setup
int p; // Variable to store potemeterPin value
const int MODE_PIN = 3; // D3 used to toggle between GPS or Pulse Signal by removing a jumper.
int MODE_STATE; // State of D3 selects GPS or Pulse Signal as speed input
const int PPM_PIN = 4; // D4 used to enable pulses per mile calibration
unsigned int PPM_STATE; // Enables calibration mode when state is LOW
int calibration = 0; // Flag used to display calibration
float SecondsPerMinute = 60.00; // Used in calculation

//Startup confing
void setup() {
delay(4000);
pinMode (MODE_PIN, INPUT_PULLUP); // Enables internal pullup resistor and sets mode pin as an input
pinMode (PPM_PIN, INPUT_PULLUP); // Enables internal pullup resistor and sets calibration pin as an input

FreqCount.begin(1000); // Starts the frequency counter and sets timer interval to 1000ms


//Initialise stepper motor
Motor.zero(); //Run motor against stops to zero
Motor.setPosition(960);
Motor.updateBlocking();
delay (500);
Motor.setPosition(0);  //0MPH
Motor.updateBlocking();
delay (500);

}

//Main loop
void loop() {


      float ConvertRPM = (PulsesPerRev * 2);

      if (FreqCount.available()){
        PulseCount = FreqCount.read();
        revolutionsPerMinute = (PulseCount * 2); // Speed is calculated from pulse frequency.
      } 
		  
      motorStep = ((StepsPerDegree * (revolutionsPerMinute * GaugeDegreesPerRPM) - dialCorrection));
        if (motorStep < 0 ){
          motorStep = 0;
        }else if (motorStep >= 0){
          motorStep = motorStep;
        }
      Motor.setPosition(motorStep); //Sets step count to move speedo needle    
      Motor.updateBlocking(); //Call this frequently to ensure the motor position is always updated.



// Testing
  Serial.print("Pulse Count:");
  Serial.println(PulseCount);
  Serial.print("MotorStep:");
  Serial.println(motorStep);
  Serial.print("RPM:");
  Serial.println(revolutionsPerMinute);
}
