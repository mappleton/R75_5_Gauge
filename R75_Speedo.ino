

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


//Define OLED module
//#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans12pt7b.h> // including font 12, 18, 24pt available


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Define GPS module
#include <SoftwareSerial.h>
static const int RXPin = 11, TXPin = 12; // Rx on Arduino needs to be connected to Tx on GPS Module. Same for Tx on Arduino conencted to Rx on GPS module.
static const uint32_t GPSBaud = 9600;
int gps_speed;
int num_sat; // Varaible used to caputre number of active GPS satellites.
SoftwareSerial ss(RXPin, TXPin);

//Define stepper motor
#include <SwitecX25.h>
const float StepsPerDegree = 3.00;  // Number of steps to move one degree
const unsigned int MaxMotorRotation = 225; // 315 max degrees of movement
const unsigned int MaxMotorSteps = MaxMotorRotation * StepsPerDegree; // Maximum number of steps motor can move
const float SpeedoDegreesPerMPH = 10.8; // 90 degrees equals 50mph on the dial. 90/50 provides degrees per mph.
const float SpeedBuffer = 1.03; // Adds 3% to GPS speed to ensure below speed limit
float motorStep = 0.00; // Number of steps to move motor
float PulsesPerMile = 1000; // Number of pulses per mile produced by input signal
float ConvertMph = 0.00; // Used in frequency to MPH conversion
unsigned int PulseCount; // Used to store pulse count captured by FreqCount
float milesPerHour = 0.00; // Variable to store current speed.
float OdoMph = 0.00; // Variable to store actual current speed for odometer calculation.
const int updateInterval = 10;  // 50 milliseconds speedo needle position update rate
unsigned long previousMillis = 0;   // last time we updated the speedo
unsigned int dialCorrection = 0; // Dial reading starts at 10mph which = 56 stepper motor steps. This number is subtracted from output to provide accurate indicated speed
SwitecX25 Motor(MaxMotorSteps, 6,7,8,9); // Declare library to be used, call it Motor and define digital pins connected to stepper coils

//Define EEPROM settings
#include <EEPROMWearLevel.h> // This library distributes writes to EEPROM to prolong life
#define EEPROM_LAYOUT_VERSION 2
#define AMOUNT_OF_INDEXES 2
#define INDEX_CONFIGURATION_VAR1 0 // Array used to store tenths of a mile reading
#define INDEX_CONFIGURATION_VAR2 1 // Array used to store miles

//Define odometer
float SecondsPerHour = 3600.00; // Used in speed calculation
float distsubtotal; // Running total of distance covered.
float TotalDistance; // Total distance covered. Stored in EEPROM at power down.
const int SpeedSamplePeriod = 1000; // Update the odometer every 500 ms.
unsigned long PreviousMillis2 = 0; // Used by odometer update function.
int satwarning_flag = 0; //Toggle to display no satellite warning.
//Define Odo test distances comment out later
float Test5digit = 10000;
float Test4digit = 1000;
float Test3digit = 100;
float Test2digit = 10;
float Test1digit = 1;

//Variables used throughout code
#define potmeterPin A1 // Used to calibrate pulses per mile during setup
int p; // Variable to store potemeterPin value
const int MODE_PIN = 3; // D3 used to toggle between GPS or Pulse Signal by removing a jumper.
int MODE_STATE; // State of D3 selects GPS or Pulse Signal as speed input
const int PPM_PIN = 4; // D4 used to enable pulses per mile calibration
unsigned int PPM_STATE; // Enables calibration mode when state is LOW
int calibration = 0; // Flag used to display calibration

//Startup confing
void setup() {
Serial.begin(9600);
delay (2000);

pinMode (MODE_PIN, INPUT_PULLUP); // Enables internal pullup resistor and sets mode pin as an input
pinMode (PPM_PIN, INPUT_PULLUP); // Enables internal pullup resistor and sets calibration pin as an input

FreqCount.begin(1000); // Starts the frequency counter and sets timer interval to 1000ms

//Intialise the OLED  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
display.display();
display.clearDisplay();
//display.setTextSize(1);      // Normal 1:1 pixel scale
display.setFont(&FreeSans12pt7b);
display.setTextColor(SSD1306_WHITE); // Draw white text
display.setCursor(0, 63.5);     // Start at top-left corner
display.println("BMW R75/5");
display.display();
delay(2000);
display.clearDisplay();

//EEPROMwl.begin(EEPROM_LAYOUT_VERSION, AMOUNT_OF_INDEXES);

//Initialise stepper motor
Motor.zero();//Run motor against stops to zero
Motor.setPosition(960);
Motor.updateBlocking();
delay (500);
Motor.setPosition(0);  //?MPH
Motor.updateBlocking();
delay (500);

calibrateppm(); // Used to calibrate pules per mile. Only triggered if switch enabled

//Read odometer values from EEPROM
EEPROMwl.get(INDEX_CONFIGURATION_VAR1, TotalDistance);
EEPROMwl.get(INDEX_CONFIGURATION_VAR2, PulsesPerMile);

ss.begin(GPSBaud);


}

//Main loop
void loop() {

  MODE_STATE = LOW; // Check status of mode jumper toggle

    if(MODE_STATE == LOW){ // Jumper connected sends D3 low to select pulse as speed input.

      float ConvertMph = (PulsesPerMile / SecondsPerHour);

      if (FreqCount.available()){
        PulseCount = FreqCount.read();
        milesPerHour = (PulseCount / ConvertMph); // Speed is calculated from pulse frequency.
        OdoMph = (PulseCount/ConvertMph);
      }

      motorStep = ((StepsPerDegree * (milesPerHour * SpeedoDegreesPerMPH) - dialCorrection));
        if (motorStep < 0 ){
          motorStep = 0;
        }else if (motorStep >= 0){
          motorStep = motorStep;
        }
      Motor.setPosition(motorStep); //Sets step count to move speedo needle
      Motor.updateBlocking(); //Call this frequently to ensure the motor position is always updated.

      //Odometer update code
      unsigned long currentMillis2 = millis();

      if (currentMillis2 - PreviousMillis2 >= SpeedSamplePeriod) {
        PreviousMillis2 = currentMillis2;
        distsubtotal = (OdoMph / SecondsPerHour);
        TotalDistance += distsubtotal;
        distsubtotal = 0;
        if (TotalDistance > 99999){
          TotalDistance = 0;
        }
      }
      if (calibration == 0){
        displayodometer();
      }
    }

  // Write odometer to EEPROM when speed drops below 3pmh
  if (milesPerHour <= 3){
    updateeeprom();
  }

// Testing
  Serial.print("Pulse Count:");
  Serial.println(PulseCount);
  Serial.print("MotorStep:");
  Serial.println(motorStep);
  Serial.print("Miles Per Hour:");
  Serial.println(milesPerHour);
}

//Functions



void updateeeprom(){ // Called from main loop when speed is below 3mph to write values to EEPROM

//  EEPROMwl.put(INDEX_CONFIGURATION_VAR1, TotalDistance);
}

// Uncomment this section and comment out above to reset all odometer values
/*
void updateeeprom(){

  EEPROMwl.put(INDEX_CONFIGURATION_VAR1, 0);
}
*/



void displayodometer() {

  display.clearDisplay();
  display.setTextSize (1);
  if (TotalDistance >= 10000) {display.setCursor(44,63.5);}
    else if (TotalDistance >= 1000) {display.setCursor(57,63.5);}
    else if (TotalDistance >= 100) {display.setCursor(69,63.5);}
    else if (TotalDistance >= 10) {display.setCursor(83,63.5);}
    else {display.setCursor(94,63.5);}

  display.setTextColor(SSD1306_WHITE); // Draw white text
//  display.print(Test5digit,1);
  display.print(TotalDistance,1);

  display.display();
}

void calibrateppm(){

  while (PPM_STATE == LOW){

    PPM_STATE = digitalRead(PPM_PIN); // Check status of calibration pin
    p = analogRead(potmeterPin); // get value from potmeter
    PulsesPerMile = map(p,0,1023,1000,6000); // map POT input between 1000 and 6000 pulses per mile.
    display.setCursor(5,6);
    display.println("PPM:");
    display.display();
    display.setCursor(45,6);
    display.println(PulsesPerMile);
    display.display();
  }
    display.clearDisplay();
//    EEPROMwl.put(INDEX_CONFIGURATION_VAR2, PulsesPerMile);
    calibration = 0;
}
