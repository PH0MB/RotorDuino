/*
 RotorDuino - Yaesu GS-232A Computer Control Interface For Antenna Rotators Emulator
 Author:    M.A. Bernards
 HamCall:   PH0MB
 Date:      2/3/2010
*/

/*
 The Yaesu GS-232A implements a simple on or 2 byte command protocol.
 Exerpt from the manual
 Returned by  [H] Command:
 ---------- COMMAND LIST 1 ----------
 R  Clockwise Rotation
 L  Counter Clockwise Rotation
 A  CW/CCW Rotation Stop
 C  Antenna Direction Value
 M  Antenna Direction Setting. MXXX
 M  Time Interval Direction Setting.
 MTTT XXX XXX XXX ---
 (TTT = Step value)
 (XXX = Horizontal Angle)
 T  Start Command in the time interval direction setting mode.
 N  Total number of setting angles in �M� mode and traced
 number of all datas (setting angles)
 X1 Rotation Speed 1 (Horizontal) Low
 X2 Rotation Speed 2 (Horizontal) Middle 1
 X3 Rotation Speed 3 (Horizontal) Middle 2
 X4 Rotation Speed 4 (Horizontal) High
 S  All Stop
 O  Offset Calibration
 F  Full Scale Calibration
 */
 
 /*
 Returned by  [H2] Command:
 ---------- HELP COMMAND 2 ----------
 U  UP Direction Rotation
 D  DOWN Direction Rotation
 E  UP/DOWN Direction Rotation Stop
 C2 Antenna Direction Value
 W  Antenna Direction Setting.
 WXXX YYY
 W  Time Interval Direction Setting.
 WTTT XXX YYY XXX YYY ---
 (TTT = Step value)
 (XXX = Horizontal Angle)
 (YYY = Elevation Angle)
 T  Start Command in the time interval direction setting mode.
 N  Total number of setting angle in �W� mode and traced
 number of all datas (setting angles)
 S  All Stop
 02 Offset Calibration
 F2 Full Scale Calibration
 B  Elevation Antenna Direction Value
 */
 /*
 The  GS-232A does not echo the commands back, but sends back a <CR>, \r or 0x0D if the command is received
 It also sends a <LF>, \n or 0x0A if the command returns data.
 
 A Bad command sends a "? >"
 
 All angles are in degrees in 3digits zero padded i.e. M001 W212 056l

 This sketch uses an old Dennard Ltd 482 camera rotor
 It consist of two Berger-Lahr RSM 63/8 24 VAC synchronous motors running 375 RPM and two 1:600 reduction boxes.
 So due to the syncronous motors the number of revolutions per degree at 50Hz you can calculate the number of net cycles needed for one degree adjustment.
 Revolutions Per Second = 375/60 = 6.25 
 Time needed for one 360 degree rotation 500 / 6.25 = 80 seconds 
 Number of revolutions needed for 1 degree 500 / 360 = 1 7/18
 Time needed for one Degree deviation 500 / 360 * 6.25 = 2/9 = 0,2222222 sec
 Number of cycles per degree deviation = NetFrequency * Reduction  * SecondsPerMinute / RPM * 360 = 50*500*60/(375 * 360) = 11,11 cycles
*/
#include <stdio.h> // for string handling stuff
#include <avr/pgmspace.h>
#include <ctype.h>
#include <Messenger.h> // use the messenger library to process the commands
// Instantiate Messenger object with the message function and the default separator 
// (the space character)
Messenger message = Messenger();

#define VERSION "RotorDuino V.1.0"
#define MotorLeft 7      // Pin high Activates SS Relay for left turn
#define MotorRight 6     // Pin high Activates SS Relay for right turn
#define MotorDown 5      // Pin high Activates SS Relay for down turn
#define MotorUp 4        // Pin high Activates SS Relay for up turn
#define MotorElevation 3 // Input interrupt 0 pin 3 counts the number of cycles the elevation motor consumes 
#define MotorAzimuth 2   // Input interrupt 1 pin 2 counts the number of cycles the azimuth motor consumes
#define NetFrequency 50  // Hz
#define Reduction 500    // 500:1 
#define RPM 375          // Motor speed 
// PulsesPerDegree (NetFrequency * Reduction * 60) / (RPM * 360) // 11,11 is not an integer
// pulses per  full revolution (NetFrequency * Reduction * 60) / RPM = 4000 cycles
#define FullRotationCycles 4000
#define HalfRotationCycles 2000
#define SerialSpeed 9600
#define MAXLENGTH 80 // this is too short for 3800 angles for the M command , should be in the range of 16K, not sure if this is used by HRD
#define BadCommandString "? >\r\n"
#define LCDRows 16
#define LCDCols 2
const char Progress[4] = {'<', '^', '>', 'v'};

char cString[MAXLENGTH];  // array that will hold command string
int iBufferIndex; // index of buffer characters rec'd
int Elevation;
int ElevationRequest;
int Azimuth;
int AzimuthRequest;
volatile int newElevationCycle; // the 50 Hz pulse ISR counters for Elevation
int ElevationCycle; // 
int AzimuthCycle;   // and Azimuth
volatile int newAzimuthCycle; // the 50 Hz pulse ISR counters for Azimuth
boolean ElevationUp; // False = Down, True = Up
boolean AzimuthCw;   // False = CCR or Left, True = CW or right
boolean DebugMe = false;        // Prints extra debug info on serial line if true

// The [H] and [H2] Command strings reside in the Program Memory to free up RAM memory
prog_char help1_00[] PROGMEM = "---------- COMMAND LIST 1 ----------";
prog_char help1_01[] PROGMEM = "R  Clockwise Rotation";
prog_char help1_02[] PROGMEM = "L  Counter Clockwise Rotation";
prog_char help1_03[] PROGMEM = "A  CW/CCW Rotation Stop";
prog_char help1_04[] PROGMEM = "C  Antenna Direction Value";
prog_char help1_05[] PROGMEM = "M  Antenna Direction Setting. MXXX";
prog_char help1_06[] PROGMEM = "M  Time Interval Direction Setting.";
prog_char help1_07[] PROGMEM = "   MTTT XXX XXX XXX ---";
prog_char help1_08[] PROGMEM = "   (TTT = Step value)";
prog_char help1_09[] PROGMEM = "   (XXX = Horizontal Angle)";
prog_char help1_10[] PROGMEM = "T  Start Command in the time interval direction setting mode.";
prog_char help1_11[] PROGMEM = "N  Total number of setting angles in [M] mode and traced";
prog_char help1_12[] PROGMEM = "   number of all datas (setting angles)";
prog_char help1_13[] PROGMEM = "X1 Rotation Speed 1 (Horizontal) Low";
prog_char help1_14[] PROGMEM = "X2 Rotation Speed 2 (Horizontal) Middle 1";
prog_char help1_15[] PROGMEM = "X3 Rotation Speed 3 (Horizontal) Middle 2";
prog_char help1_16[] PROGMEM = "X4 Rotation Speed 4 (Horizontal) High";
prog_char help1_17[] PROGMEM = "S  All Stop";
prog_char help1_18[] PROGMEM = "O  Offset Calibration";
prog_char help1_19[] PROGMEM = "F  Full Scale Calibration";
prog_char help1_20[] PROGMEM = "H  This help list: COMMAND LIST 1";
prog_char help1_21[] PROGMEM = "";

prog_char help2_00[] PROGMEM = "---------- HELP COMMAND 2 ----------";
prog_char help2_01[] PROGMEM = "U  UP Direction Rotation";
prog_char help2_02[] PROGMEM = "D  DOWN Direction Rotation";
prog_char help2_03[] PROGMEM = "E  UP/DOWN Direction Rotation Stop";
prog_char help2_04[] PROGMEM = "C2 Antenna Direction Value";
prog_char help2_05[] PROGMEM = "W  Antenna Direction Setting.";
prog_char help2_06[] PROGMEM = "   WXXX YYY";
prog_char help2_07[] PROGMEM = "W  Time Interval Direction Setting.";
prog_char help2_08[] PROGMEM = "   (TTT = Step value)";
prog_char help2_09[] PROGMEM = "   (XXX = Horizontal Angle)";
prog_char help2_10[] PROGMEM = "   (YYY = Elevation Angle)";
prog_char help2_11[] PROGMEM = "T  Start Command in the time interval direction setting mode.";
prog_char help2_12[] PROGMEM = "N  Total number of setting angles in [W] mode and traced";
prog_char help2_13[] PROGMEM = "   number of all datas (setting angles)";
prog_char help2_14[] PROGMEM = "S  All Stop";
prog_char help2_15[] PROGMEM = "02 Offset Calibration";
prog_char help2_16[] PROGMEM = "F2 Full Scale Calibration";
prog_char help2_17[] PROGMEM = "B  Elevation Antenna Direction Value";
prog_char help2_18[] PROGMEM = "H2 This help list: HELP COMMAND 2";
prog_char help2_19[] PROGMEM = "Q  Toggle Debug (!)";
prog_char help2_20[] PROGMEM = "P  Antenna Elevation Setting PYYY(!)";
prog_char help2_21[] PROGMEM = "";

PROGMEM const char *HELP1[] = 	   // [H] Commmand Array
{
  help1_00,
  help1_01,
  help1_02,
  help1_03,
  help1_04,
  help1_05,
  help1_06,
  help1_07,
  help1_08,
  help1_09,
  help1_10,
  help1_11,
  help1_12,
  help1_13,
  help1_14,
  help1_15,
  help1_16,
  help1_17,
  help1_18,
  help1_19,
  help1_20,
  help1_21
};

PROGMEM const char *HELP2[] = 	   // [H2] Commmand Array
{
  help2_00,
  help2_01,
  help2_02,
  help2_03,
  help2_04,
  help2_05,
  help2_06,
  help2_07,
  help2_08,
  help2_09,
  help2_10,
  help2_11,
  help2_12,
  help2_13,
  help2_14,
  help2_15,
  help2_16,
  help2_17,
  help2_18,
  help2_19,
  help2_20,
  help2_21
};

char buffer[MAXLENGTH];

/* The circuit:
 * LCD RS pin to digital pin 13
 * LCD Enable pin to digital pin 12
 * LCD D4 pin to digital pin 11
 * LCD D5 pin to digital pin 10
 * LCD D6 pin to digital pin 9
 * LCD D7 pin to digital pin 8
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 */


#include <LiquidCrystal.h>
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

void setup()
{
  pinMode(MotorLeft, OUTPUT);
  pinMode(MotorRight, OUTPUT);
  pinMode(MotorUp, OUTPUT);
  pinMode(MotorDown, OUTPUT);
  pinMode(MotorAzimuth, INPUT);
  pinMode(MotorElevation, INPUT);
  Serial.begin(SerialSpeed);  // connect to the serial port and sends a bootup message.
  message.attach(messageCompleted); // and attach the messenger object function to the serial input
  Serial.print(VERSION);
  Serial.println(" Ready");
  Serial.println("Copyright (c) 2010 by PH0MB");
  // Sends a startup message to the LCD screen.
  lcd.begin(LCDRows, LCDCols);
  lcd.setCursor(0,0);
  lcd.print(VERSION);
  lcd.setCursor(0,1);
  lcd.print("(c)2010 by PH0MB");
  delay(3000); // show the hello
  //Help1();
  //Help2();
  //InititalLCDPosition();
  // Reset the elevation and azimuth to the 0 degrees by turning the elavation and azimuth left and down until it stops
  // Pin 2 and Pin 3 are normally high zo count when one 50 Hz period was active before incrementing the pulse counter
  // Not sure if the ISR can service two simultaneious changes, so one ISR gets the rising and one the falling edge
 
  attachInterrupt(0, updateAzimuthCycle, RISING );
  attachInterrupt(1, updateElevationCycle, FALLING );
  lcd.setCursor(0,0);
  ResetToZeroDegrees();
  InititalLCDInfo();
}

/*
 * Main Loop
 */

void loop() {
  while ( Serial.available()) message.process(Serial.read()); // feed the messenger :-)
  // Check if the Azimuth motor is running
  if (newAzimuthCycle != AzimuthCycle) {
    AzimuthCycle = newAzimuthCycle;
    Azimuth = CyclesToDegrees(AzimuthCycle);
    updateLCDAzimuth(Azimuth);
    if (DebugMe) {
      Serial.print(Azimuth);
      Serial.print("\r");
    }
  }
  // And also for the elevation motor
  if (newElevationCycle != ElevationCycle) {
    ElevationCycle = newElevationCycle;
    Elevation = CyclesToDegrees(ElevationCycle);
    updateLCDElevation(Elevation);
    if (DebugMe) {
      Serial.print(Elevation);
      Serial.print("\r");
    }
  }
  if (Azimuth == AzimuthRequest && AzimuthRequest != -1) {
    if (DebugMe) {
      Serial.println(Azimuth);
      Serial.println(AzimuthRequest);
    }
    StopAzimuth();
    AzimuthRequest = -1;
  }
  if (Elevation == ElevationRequest && ElevationRequest != -1) {
    if (DebugMe) { 
      Serial.println(Azimuth);
      Serial.println(AzimuthRequest);
    }
    StopElevation();
    ElevationRequest = -1; // Reset the request value
  }
  delay(5); // wait a while before checking the next cycle
}

void messageCompleted() {
// This loop will echo each element of the message separately
  while ( message.available() ) {
    message.copyString(cString,MAXLENGTH);
    if (DebugMe) {
      Serial.println(cString); // for debugging purposes, return the command string
    }
    ParseCommand();
    Serial.print("\r"); // this one must be done after the command
  }  // End While
}

void ParseCommand() {

  switch (toupper(cString[0])) {
    case 'A': // CW/CCW Rotation Stop
      StopAzimuth();
      break;

    case 'B': // Elevation Antenna Direction Value
      //Serial.println(Elevation);
      snprintf(buffer, 10, "+%04d\n", Elevation);
      Serial.print(buffer);
      break;
      
    case 'C': // Antenna Direction Value
      if (cString[1] == '2') {
        snprintf(buffer, 10, "+%04d", Azimuth);
        Serial.print(buffer);
        snprintf(buffer, 10, "+%04d\n", Elevation);
        Serial.print(buffer);
      } else {
        snprintf(buffer, 10, "+%04d\n", Azimuth);
        Serial.print(buffer);
      }
      break;
      
    case 'D': //  D DOWN Direction Rotation
      RotateDown();
      break;
      
    case 'E': // UP/DOWN Direction Rotation Stop
      StopElevation();
      break;
      
    case 'F': // Full Scale Calibration {,2}
      ResetToZeroDegrees();
      break;
      
    case 'H': // Help {,2}
      if (cString[1] == '2') {
        Help2();
      } else {
        Help1();
      }
      break;
      
    case 'L': // Counter Clockwise Rotation
      RotateLeft();
      break;
      
    case 'M': // Antenna Direction Setting. MXXX
      cString[0] = '0';
      AzimuthRequest = atoi(cString);
      RotateAzimuth(AzimuthRequest);
      break;
      
    case 'N': // Total number of setting angles in [M] or [W] mode and traced number of all datas (setting angles)
      Serial.print("0\n"); // not implemented yet, return a bogus value
      break;
      
    case 'O': // Offset Calibration
      ResetToZeroDegrees();
      break;
      
    case 'P': // This is a unofficial command not listed in the Yaesu Just Elevation
      cString[0] = '0';
      ElevationRequest = atoi(cString); //  value is number after P
      RotateElevation(ElevationRequest);
      break;
      
    case 'Q': // This is a unofficial command not listed in the Yaesu
      DebugMe = !DebugMe; //Toggle the Debug Flag
      if (DebugMe) {
        Serial.print("Debug On\n");
      }
      break;
      
    case 'R': // Clockwise Rotation
      RotateRight();
      break;
      
    case 'S': // All Stop
      StopRotate();
      break;
      
    case 'T': // Start Command in the time interval direction setting mode.
      // Does nothing yet
      break;

    case 'U': // UP Direction Rotation
      RotateUp();
      break;
  
    case 'V': // UP Direction Rotation
      Serial.println(VERSION);
      break;
      
    case 'W': // Antenna Direction Setting.
      cString[0] = ' ';
      AzimuthRequest = atoi(cString); // pan value is number after W
      message.copyString(cString,MAXLENGTH); // pull the second argument from the message stack
      ElevationRequest = atoi(cString);
      RotateAzimuth(AzimuthRequest);
      RotateElevation(ElevationRequest);
      break;
      
    case 'X': // Rotation Speed {1,2,3,4}
      // Does nothing
      break;
    
    default: 
      // if nothing else matches, do the default
      // default is optional
      BadCommand();
      
    } // End Switch 
}
// ISR for Pin 2 and 3 (ISR0 and ISR1) Interrupt Service Routines

void updateAzimuthCycle() {
  if (AzimuthCw) {
    newAzimuthCycle++;
  } else {
    newAzimuthCycle--;
  }
}

void updateElevationCycle() {
  if (ElevationUp) {
    newElevationCycle++;
  } else {
    newElevationCycle--;
  }
}
// End of ISR functions

void Help1() {
  for (int i = 0; i < 21; i++)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(HELP1[i]))); // Necessary casts and dereferencing, just copy. 
    Serial.println( buffer );
    delay( 60 );
  }
}
// 
void Help2() {
  for (int i = 0; i < 21; i++)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(HELP2[i]))); // Necessary casts and dereferencing, just copy. 
    Serial.println( buffer );
    delay( 60 );
  }
}


// Truth table for motor pins
// M{L,D} M{R,U} Action
// 0      0  All Off
// 1      0  Turn Left/Down
// 0      1  Turn Right/Up
// X      X  Forbidden State

void StopRotate() {
  if (DebugMe) Serial.println("StopRotate");
  StopElevation();
  StopAzimuth();
}

void StopElevation() {
  // Let the last degree finish and then stop
  if (DebugMe) Serial.println("StopElevation");
  digitalWrite(MotorUp, LOW);
  digitalWrite(MotorDown, LOW);
}

void StopAzimuth() {
  // Let the last degree finish and then stop
  if (DebugMe) Serial.println("StopAzimuth");
  digitalWrite(MotorLeft, LOW);
  digitalWrite(MotorRight, LOW);
}

void RotateAzimuth(int Degrees) {
  if ( Degrees < 360 && Degrees >= 0 ) {
    // This is a valid request
    AzimuthRequest = Degrees;
    if (DebugMe) {
      Serial.print("current azimuth = ");
      Serial.println(Azimuth);
    }
    if (Degrees > Azimuth) {
      if (DebugMe) {
        Serial.print("rotating right to ");
        Serial.println(Degrees);
      }
      RotateRight();
    } else {
      if (DebugMe) {
        Serial.print("rotating left to ");
        Serial.println(Degrees);
      }
      RotateLeft();
    }
  } else {
    BadCommand();
  }  
}

void RotateElevation(int Degrees) {
  if ( Degrees < 180 && Degrees >= 0 ) {
    // This is a valid request
    ElevationRequest = Degrees;
    if (DebugMe) {
      Serial.print("current elevation = ");
      Serial.println(Elevation);
    }
    if (Degrees > Elevation) {
      if (DebugMe) {
        Serial.print("rotating up to ");
        Serial.println(Degrees);
      }
      RotateUp();
    } else {
      if (DebugMe) {
        Serial.print("rotating down to ");
        Serial.println(Degrees);
      }
      RotateDown();
    }
  } else {
    BadCommand();
  }  
}

void RotateLeft() {
  AzimuthCw = false;
  digitalWrite(MotorRight, LOW);
  digitalWrite(MotorLeft, HIGH);
}

void RotateRight() {
  AzimuthCw = true;
  digitalWrite(MotorLeft, LOW);
  digitalWrite(MotorRight, HIGH);
}

void RotateUp() {
  ElevationUp = true;
  digitalWrite(MotorDown, LOW);
  digitalWrite(MotorUp, HIGH);
}

void RotateDown() {
  ElevationUp = false;
  digitalWrite(MotorUp, LOW);
  digitalWrite(MotorDown, HIGH);
}

void ResetToZeroDegrees() {
  int i = 0;
  // local vars
  lcd.setCursor(0,0);
  lcd.print("Resetting...    ");
  lcd.setCursor(0,1);
  lcd.print("Rotor Position  ");
  Serial.println("Resetting Rotor Position....");
  ElevationUp = false;
  AzimuthCw = false;
  newElevationCycle = 30000; // just a bogus big value
  newAzimuthCycle = 30000;   // just a bogus big value
  RotateLeft();
  RotateDown();

  // Since we are resetting the position again for both azimuth and elevation
  // I need to be sure the motors have stopped both. 
  // Just check if the counters don't decrement any more
  
  ElevationCycle = 30000; // just a bogus big value 
  AzimuthCycle = 30000;   // just a bogus big value
  
 do { // Lets it and wait until the motors have all turned down and left and the tick counters do not change any more
    lcd.setCursor(15,0);
    lcd.print(Progress[i++]);
    if (i > 3) {i = 0;}
    AzimuthCycle = newAzimuthCycle;
    ElevationCycle = newElevationCycle;
    delay(1000); // wait a second before next trial to measure the tick counters
  } while ((AzimuthCycle != newAzimuthCycle) || (ElevationCycle > newElevationCycle));
  StopRotate();
  // reset all values to zero or initial values
  Elevation = 0;
  ElevationRequest = -1; // No request
  Azimuth = 0;
  AzimuthRequest = -1;
  newElevationCycle = 0; // the 50 Hz pulse ISR counters for Elevation
  ElevationCycle = 0; // 
  AzimuthCycle = 0;   // and Azimuth
  newAzimuthCycle = 0; // the 50 Hz pulse ISR counters for Azimuth
  lcd.setCursor(0,0);
  lcd.print("Resetting.. Done");
  lcd.setCursor(0,1);
  lcd.print("Rotor Position  ");
  Serial.println("Resetting Rotor Position.... Done");
  Serial.print("Azimuth = ");
  Serial.println(Azimuth);
  Serial.print("Elevation = ");
  Serial.println(Elevation);
  delay(2000);
  InititalLCDInfo();
}

void InititalLCDInfo() {
  // lcd cursor adresses as (column, row)
  lcd.setCursor(0,0); // First Line
  lcd.print("Azimuth:    0  ");  
  lcd.setCursor(15,0);
  lcd.print((char)223); // need a 223 behind for the degree ; 
  lcd.setCursor(0,1); // Second Line
  lcd.print("Elevation:  0  ");
  lcd.setCursor(15,1);
  lcd.print((char)223); // need a 223 behind for the degree
}

void updateLCDAzimuth(int Azimuth) {
  lcd.setCursor(12,0);
  lcd.print("   ");   // clear the value
  lcd.setCursor(12,0);
  lcd.print(Azimuth); // Maybe need zero padding routine
  lcd.setCursor(15,0);
  lcd.print((char)223); // 223 Degree 
}

void updateLCDElevation(int Elevation) {
  lcd.setCursor(12,1);
  lcd.print("   ");   // clear the value
  lcd.setCursor(12,1);
  lcd.print(Elevation); // Maybe need zero padding routine
  lcd.setCursor(15,1);
  lcd.print((char)223); // need a 223 behind for the degree
}

void BadCommand() {
  Serial.println( BadCommandString );
  Serial.flush();
  cString[0] = '\0'; // Zero string
}

void Done() {
  Serial.println("\r"); // just sends a <CR><LF> to ACK
}

int DegreesToCycles(int Degrees) {
  int result;
  result = map(Degrees,0, 360, 0, FullRotationCycles);
  return result;
}

int CyclesToDegrees(int Cycles) {
  int result;
  result = map(Cycles ,0, FullRotationCycles ,0, 360);
  return result;
}
// End of RotorDuino
