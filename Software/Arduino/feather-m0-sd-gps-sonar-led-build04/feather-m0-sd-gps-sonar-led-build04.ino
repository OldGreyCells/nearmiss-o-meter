
/** *************************************************************************** 
 *  Arduino code for the nearmiss-o-meter. More information here:
 *  
 *  http://oldgreycells.blogspot.com
 *  
 *  Written for an Adafruit Feather M0 Adalogger but this sketch should be adaptable
 *  to other Arduinos and sonars.
 *  
 *  HISTORY
 *  2017/02/08 - Initial release.
 *  
 */


/** *************************************************************************** 
 * The MIT License (MIT)
 * 
 * Copyright (c) 2016 oldgreycells@gmail.com
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights 
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
 * copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 * 
 */
/** ***************************************************************************
 *  ****** ACTION REQUIRED: UPDATE THE SERIAL_BUFFER_SIZE *********************
 * 
 * The ublox GPS delivers all NMEA messages at once which can quickly overflow
 * the default 64 byte Serial buffer. The board specific core Arduino code is
 * compiled before the sketch, so we cannot set the serial buffer here.
 * For Adafruit Feather M0 the serial buffer size is defined here:
 * ~/.arduino15/packages/adafruit/hardware/samd/1.0.13/cores/arduino/RingBuffer.h
 * Change this line:
 * #define SERIAL_BUFFER_SIZE 64
 * to:
 * #define SERIAL_BUFFER_SIZE 256
 * This will need to be changed each time the board definition is updated.
 * The location (e.g. 1.0.13) and/or name of the #define may also change!
 */
// ----------------------------------------------------------------------------
// Arduino standard libraries (installed with IDE)
// ----------------------------------------------------------------------------
// Required for SERCOM (additional serial ports) on M0
#include <Arduino.h>
#include "wiring_private.h"

/*  SdFat Card library
 *   https://github.com/greiman/SdFat
 *
 *   I don't think this is necessary if not using SPI for other peripherals
 *   but am leaving this here for future reference:
 *   Modify \SdFat\SdFatConfig.h and change line 81 to read;
 *   #define SD_SPI_CONFIGURATION 1
 *   If SD_SPI_CONFIGURATION is defined to be one, SdFat uses the standard
 *   Arduino SPI.h library, as opposed to the custom implementation within
 *   the SdFat library.
 */
#include <SdFat.h>

/* Time library
 * https://github.com/PaulStoffregen/Time v1.5
 */
#include <Time.h>

/* The TinyGPS Plus library
 * https://github.com/mikalhart/TinyGPSPlus
 * or
 * http://arduiniana.org/libraries/tinygpsplus/
 */
#include <TinyGPS++.h>

/* Serial output Ultrasonic sonar library
 * https://github.com/OldGreyCells/SerialPing
 */
#include <SerialPing.h>


// Constants


// Fixed pin allocation (for Adafruit Feather M0). Do not change these.

// This is allocated by the SD library.
const byte SD_CS_PIN = 4; 
// Used to measure battery voltage.
const byte BATTERY_VOLTAGE_PIN = A7;


const byte SONAR_OFFSET = 0; //Default sonar offset
const int GPS_TIME_UPDATE = 5000;
// Milliseconds between sonar pings.
const int SONAR_HEARTBEAT = 250;
// Number of milliseconds to keep recording after GPS fix is lost.
const unsigned int GPS_STALE_LIMIT = 5000;
// Horizontal Dilution of Precision (GPS accuracy)
const unsigned int GPS_HDOP_LIMIT = 500;
// Milliseconds between actually writing records to SD card. More frequent will use more battery.
const unsigned int SD_FLUSH_INTERVAL = 2000;


// Pin allocation. If you've made errors with the wiring up, you may be able to fix it here!
//const byte SONAR_TX_PIN = 10;//5; // The trigger pin (or TX for SoftwareSerial)
//const byte SONAR_RX_PIN = 11;//6; // The RX ping (for US100/SoftwareSerial)
const byte SONAR_TX_PIN = 11;//20;//5; // The trigger pin (or TX for SoftwareSerial/SERCOM)
const byte SONAR_RX_PIN = 10;//21;//6; // The RX ping (for US100/SoftwareSerial/SERCOM)
//const byte SONAR_PW_PIN = 9;//6; // The echo ping (for MaxSonar)

const byte LDR_PIN = A2; // Used to measure light level.
const byte SWITCH_PIN = A5;//10;
const byte GPS_LED = 6; //red
const byte RECORD_LED = 5; //green
const byte NEAR_MISS_LED = A3; //12; //blue

//Green LED is much brighter than red led, so balance with this...
const byte GPS_LED_PERCENT = 100; //red
const byte RECORD_LED_PERCENT = 70; //green

// Delimiter for the CSV records.
const char CSV_DELIMITER = ',';
// Delimiter for nearmiss.cfg.
const char CONFIG_DELIMITER = '=';
// Max number of pings required to get the median.
const byte SONAR_MEDIAN_ITERATIONS = 5;
//Maximum milliseconds to wait for all pings in median
const byte SONAR_MEDIAN_TIMEOUT = 90; //Milliseconds;
// Max distance (in cm) to read. Greater than this value will record 0.
const int SONAR_MAX_DISTANCE = 200;
// Distance in cm that counts as a near miss. Only used for led indication. Max 255.
const byte NEAR_MISS_CM = 100;
// Number of clear (>NEAR_MISS_CM) pings before led of turned off.
const byte NEAR_MISS_LED_PINGS = 5;
// Duration of tag in milliseconds
const int TAGGING_DURATION = 3000;
// Battery voltage limit that will trigger LED warning, in millivolts.
const int LOW_VOLTAGE_LIMIT = 2800;
const int VOLTAGE_CHECK_FREQ = 60000; //6000 Every minute

const int LDR_MIN = 10;
const int LDR_MAX = 1024;
const int LED_MIN = 20;
const int LED_MAX = 255;

// Momentary (press button) switch handling
const byte SWITCH_DEBOUNCE_TIME = 15; //Increase this if tag toggling is reading bounces
const int SWITCH_SHORT_PRESS_MAX = 2000; // Also minimum for medium press...
const int SWITCH_LONG_PRESS_MIN = 10000; // and also maximum for medium press.

// Used as status values and for LED flash frequency
const byte GPS_NO_FIX    = B11111111; // Continuous
const byte GPS_HAVE_FIX  = B00000100; // Once per second
const byte GPS_SERIAL_ERROR  = B01010101; // SERIAL_BUFFER_SIZE not 256
const byte RECORD_NO_SD  = B11111111; // Continuous
const byte RECORD_ERROR  = B11101110; // Long flash, twice per second
const byte RECORDING     = B01000000; // Once per second
const byte TAGGING       = B01010101; // Four times second (in combination with RECORDING)
const byte NEAR_MISS     = B10101010; // Four times per second
const byte SONAR_TEST    = B10000000; // One short flash per second
const byte LOW_BATTERY   = B01010101; // Four times per second OR'd with GPS_HAVE_FIX

const char CONFIG_FILE_NAME[] = "/nearmiss.cfg";
const int MAX_LINE_LENGTH=80;
const char DATA_DIRECTORY[] = "/data";

const char CONFIG_SONAR_OFFSET[] = "SONAR_OFFSET";
const char CONFIG_TIMEZONE[] = "TIMEZONE";
const char CONFIG_SONAR_HEARTBEAT[] = "SONAR_HEARTBEAT";
const char CONFIG_GPS_STALE_LIMIT[] = "GPS_STALE_LIMIT";
const char CONFIG_SD_FLUSH_INTERVAL[] = "SD_FLUSH_INTERVAL";
const char CONFIG_GPS_LED_PERCENT[] = "GPS_LED_PERCENT";
const char CONFIG_RECORD_LED_PERCENT[] = "RECORD_LED_PERCENT";



// Config variables that can be overwritten in nearmiss.cfg

int gpsTimeUpdate = GPS_TIME_UPDATE;

// Distance from sonar to elbow.
// Change this in nearmiss.cfg if your nearmiss-o-meter is set nearer the centre of the bike.
unsigned int sonarOffset = SONAR_OFFSET;
unsigned int sonarHeartbeat = SONAR_HEARTBEAT;
// Offset hours from gps time (UTC)
float timezone = 0;
// Used for timestamp, updated by timezone in nearmiss.cfg
char isoTimezone[7] = "+00:00";

// GPS
unsigned int gpsStaleLimit = GPS_STALE_LIMIT;
unsigned int gpsHdopLimit = GPS_HDOP_LIMIT;

// Milliseconds between actually writing records to SD card. More frequent will use more battery.
unsigned int sdFlushInterval = SD_FLUSH_INTERVAL;

byte gpsLedPercent = GPS_LED_PERCENT;
byte recordLedPercent = RECORD_LED_PERCENT;



// Variables
//The millisecond the gps reports (at .00 seconds)
long gpsEpoch = 0;
// Set at start of each loop for simple state machine.
unsigned long loopStart = 0;
// The last time the clock was updated
unsigned long previousTimeUpdate = 0;
// The last time sonar was pinged.
unsigned long previousPing = 0;
// The last time the SD card was flushed
unsigned long previousFlush = 0;
// The last time the battery voltage was checked
unsigned long previousVoltageCheck = 0;



// Sonar variables
byte sonarCm = 0;
byte nearmissCounter = 0;

// Switch handling
int switchDownTime = 0;
byte switchState = HIGH;
unsigned long switchLoopTime = 0;
boolean switchPressed = false;

// Used for status and for led flashing
byte gpsState = 0;
byte recordState = 0;
byte tagState = 0;
byte nearMissState = 0;
byte batteryState = 0;
byte testState = 0;//SONAR_TEST;
unsigned long tagStart = 0;

// Timers for LED flashing
unsigned long ledTimer = 0;
byte ledTimePos = 0;
byte ledLevel = 255;
byte gpsLedLevel = ledLevel*gpsLedPercent/100;
byte recordLedLevel = ledLevel*recordLedPercent/100;



// Required for SERCOM (additional serial ports) on M0
// pinPeripheral() function
//Uart Serial2 (&sercom1,SONAR_RX_PIN,SONAR_TX_PIN, SERCOM_RX_PAD_0, UART_TX_PAD_2);
Uart Serial2 (&sercom1,SONAR_RX_PIN,SONAR_TX_PIN, SERCOM_RX_PAD_2, UART_TX_PAD_0);
void SERCOM1_Handler() {
  Serial2.IrqHandler();
}

// The TinyGPS++ object
TinyGPSPlus gps;
SdFat SD;
// Config file instance.
SdFile configFile;
// Logging file instance.
SdFile logFile;

// Create a SerialPing instance
SerialPing sonar(Serial2, SONAR_MAX_DISTANCE);


/** **********************************************************************
 * Setup
 *
 */
void setup() {
  // Initialise all the pins to their correct mode.
  pinMode(LDR_PIN, INPUT);
  pinMode(GPS_LED, OUTPUT);
  pinMode(RECORD_LED, OUTPUT);
  pinMode(NEAR_MISS_LED, OUTPUT);
  // The momentary switch connects to ground when pressed so use internal resistor to pull up.
  // (saves having a resistor on the circuit board)
  pinMode(SWITCH_PIN, INPUT_PULLUP);


  Serial.begin(9600);//115200);
  Serial1.begin(9600);
  Serial2.begin(9600);
  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(SONAR_TX_PIN, PIO_SERCOM);
  pinPeripheral(SONAR_RX_PIN, PIO_SERCOM);
  delay(2000); //Give me chance to turn on serial monitor.

  // Turn all the LEDs off.
  analogWrite(GPS_LED, 0);
  analogWrite(RECORD_LED, 0);
  digitalWrite(NEAR_MISS_LED, LOW);
  setLedLevels();

  //Turn off superfluous NMEA messages. All except GGA & RMC
  //are not used by TinyGPS++ but will fill the Serial1 buffer
  //Serial1.print(F("$PUBX,40,GGA,0,0,0,0*5A\r\n"));
  Serial1.print(F("$PUBX,40,GGA,1,1,1,0*5B\r\n")); //Required for HDOP
  Serial1.print(F("$PUBX,40,GLL,0,0,0,0*5C\r\n"));
  //Serial1.print(F("$PUBX,40,RMC,0,0,0,0*47\r\n"));
  Serial1.print(F("$PUBX,40,RMC,1,1,1,0*46\r\n"));
  Serial1.print(F("$PUBX,40,GSA,0,0,0,0*4E\r\n"));
  Serial1.print(F("$PUBX,40,GSV,0,0,0,0*59\r\n"));
  Serial1.print(F("$PUBX,40,VTG,0,0,0,0*5E\r\n"));
  Serial1.print(F("$PUBX,40,GRS,0,0,0,0*5D\r\n"));
  Serial1.print(F("$PUBX,40,GST,0,0,0,0*5B\r\n"));
  Serial1.print(F("$PUBX,40,ZDA,0,0,0,0*44\r\n"));


  //Serial.println("Checking SD card...");
  SdFile::dateTimeCallback(dateTime);
  // Check to see if an SD card is inserted.
  if (SD.begin(SD_CS_PIN, SPI_FULL_SPEED)) {
    // If we have an SD card...
    if ( SD.exists(CONFIG_FILE_NAME) ) {
      readConfig();
    } else {
      writeConfig();
    }
    SD.mkdir(DATA_DIRECTORY);
    if (SD.chdir(DATA_DIRECTORY)) {
      for (byte i = 0; i < 5; i++) {
        // ... flash the record LED twice.
        //Serial.print("Led: ");
        //Serial.print(i%2);
        //Serial.print(" : ");
        //Serial.println(recordLedLevel);
        analogWrite(RECORD_LED, i%2 ? recordLedLevel:0);
        delay(500); // wait for a half a second
      }
    } else {
      recordState = RECORD_ERROR;
    }
    //Serial.println("Have SD card");
  } else {
    // No SD card, so we can't record.
    recordState = RECORD_NO_SD;
    //Serial.println("Missing SD card");
  }
  delay(500);
//  if ( recordState == 0 ) {
//    readConfig();
//  }
  if ( SERIAL_BUFFER_SIZE != 256) {
    gpsState=GPS_SERIAL_ERROR;
  } else {
    // We don't yet have a GPS fix...
    gpsState = GPS_NO_FIX;
  } 

}

/** **********************************************************************
 * Loop
 *
 */
void loop() {
  loopStart = millis();
  feedGps();
  if (loopStart - previousPing >= sonarHeartbeat
     && ( recordState == RECORDING
        || testState == SONAR_TEST
        )
     ) {
    previousPing = loopStart;
    pingSonar();
    if ( recordState == RECORDING ) { //Alway record, even if GPS fix is lost
      writeCsvToFile();
    }
    if ( testState == SONAR_TEST ) {
      writeCsvToSerial();
    }
  }
  manageLeds();
  readSwitchState();
  checkTagState();
  checkBatteryVoltage();
}

void feedGps() {
  while (Serial1.available()) {
    //Serial.write(Serial1.peek());
    if ( gps.encode(Serial1.read())) {
      //GPS defaults to 1Hz so we take this as epoch, although encode takes 0.1 seconds...
      gpsEpoch = millis();
    }
    if ( gpsState != GPS_SERIAL_ERROR ) {
      int hdop = gps.hdop.value();
      //int fixAge = gps.location.age();
      if ( hdop > 0 && hdop < gpsHdopLimit
          && gps.location.age() < GPS_STALE_LIMIT ) {
        gpsState = GPS_HAVE_FIX;
        if ( ( loopStart - previousTimeUpdate >= gpsTimeUpdate || previousTimeUpdate == 0 ) && gps.time.isUpdated()) {
          previousTimeUpdate = loopStart;
          // set the Time to the latest GPS reading
          setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
          adjustTime(timezone * SECS_PER_HOUR);
        }
      } else {
        gpsState = GPS_NO_FIX;
      }
    }
  }
}

void checkTagState() {
  if (tagState == TAGGING && loopStart - tagStart > TAGGING_DURATION) {
    //Serial.print(loopStart);
    //Serial.print(" - ");
    //Serial.print(tagStart);
    //Serial.print(" = ");
    //Serial.println( loopStart - tagStart);
    tagState = 0;
  }

}

/** ***************************************************************************
 * Read median distance with the ultrasonic sonar.
 */
void pingSonar() {
  // Remember the time of this ping
  previousPing = loopStart;
  // Get the median microseconds and converto to cm.
  sonarCm = sonar.convert_cm(sonar.ping_median(SONAR_MEDIAN_ITERATIONS, SONAR_MEDIAN_TIMEOUT));
  //Serial.print("Sonar: ");
  //Serial.print(sonarCm);
  //Serial.println("cm");
  // If sonarCm is not 0, deduct the SONAR_OFFSET.
  sonarCm = (sonarCm)?sonarCm-sonarOffset:0;
  // Flash the blue LED if a near miss occurs.
  // This is also used to determine if maxsonar has barfed.
  if (sonarCm != 0 && sonarCm <= NEAR_MISS_CM ) {
    nearMissState = NEAR_MISS;
    // Reset the counter because were still having a near miss.
    nearmissCounter = 0;
  }
  nearmissCounter++;
  // We're in the clear for more than NEAR_MISS_LED_PINGS so...
  if (nearmissCounter > NEAR_MISS_LED_PINGS ) {
    // ...set the state to zero.
    nearMissState = 0;
  }
}

/** ***************************************************************************
 * Measure the length of time the switch is pressed and take required action
 * when released. Also acts as a debouncer.
 */
void readSwitchState() {
  if ( loopStart != switchLoopTime ) {
    switchState = digitalRead(SWITCH_PIN);
    if ( switchState == LOW ) { //Switch has been pressed
      // There is a small risk of incremental bounces adding up, but this has not shown up in testing.
      // switchLoopTime is initialised as 0, so deduct 'now' on first loop.
      switchDownTime += loopStart-(switchLoopTime?switchLoopTime:loopStart);
    }
    if ( switchDownTime > SWITCH_DEBOUNCE_TIME ) {
      switchPressed = true;
      if ( switchDownTime > SWITCH_SHORT_PRESS_MAX
         && ( ( recordState==0
            && gpsState==GPS_HAVE_FIX )
            || recordState==RECORDING
            )
        )  {
        analogWrite(RECORD_LED, recordLedLevel);
      }
      if ( switchDownTime > SWITCH_LONG_PRESS_MIN )  {
        analogWrite(RECORD_LED, 0);
        digitalWrite(NEAR_MISS_LED, HIGH);
      }
    }
    if ( switchPressed && switchState == HIGH ) { //Switch has been released
      //Serial.println("Switch pressed for: " + String(ms));
      if ( switchDownTime < SWITCH_SHORT_PRESS_MAX ) { // Short press
        //Serial.println("Short press");
        if ( recordState == RECORDING || testState!=0 ) { // Only allow tagging if recording.or testing
          //tagState = (tagState==0)?TAGGING:0; // Toggle the tag state
          tagState = TAGGING; // Turn on the tag state
          tagStart = loopStart;
        }
      } else if ( switchDownTime > SWITCH_LONG_PRESS_MIN ) { // Long press
        if ( testState==0 ) {
          testState=SONAR_TEST;
        } else {
          testState = 0;
          nearMissState = 0; // Need to turn this off here (or have near miss check timeout in main loop).
        }
      } else { // Medium press
        // Always allow recording to be turned off, even if no GPS fix.
        if (recordState == RECORDING ) {
          recordState = 0;
          tagState = 0;
          nearMissState = 0;
          testState = 0;
        } else if (recordState==0 && gpsState==GPS_HAVE_FIX) { //Only start recording if no error & have GPS fix
          recordState = RECORDING;
          testState = 0;
        }
      }
      switchDownTime = 0;
      switchPressed = false;
    }
    switchLoopTime = loopStart;
  }
}


/** **********************************************************************
 * Time returns seconds only, so calculate hundredts from GPS second epoch.
 */
byte hundredths() {
  return ((millis() - gpsEpoch)%1000)/10;
}

/** **********************************************************************
 *
void csv(Print& s) {
  if (gps.time.isUpdated() ) { //Probably don't need to update every second...
    // set the Time to the latest GPS reading
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
    adjustTime(timezone * SECS_PER_HOUR);
  }
  char isoDateTime[23];
  // Format the date and time to ISO 8601 standard.
  sprintf(isoDateTime, "%04d-%02d-%02dT%02d:%02d:%02d.%02d%s", year(), month(), day(), hour(), minute(), second(), hundredths(),isoTimezone);
    s.print(isoDateTime);
    s.print(CSV_DELIMITER);
    s.print(gps.location.lng(),6);
    s.print(CSV_DELIMITER);
    s.print(gps.location.lat(),6);
    s.print(CSV_DELIMITER);
    s.print(gps.speed.kmph(),0);
    s.print(CSV_DELIMITER);
    s.print(gps.course.deg(),0);
    s.println();
*/


/** **********************************************************************
 * Read the nearmiss.cfg file.
 */
void readConfig() {
  //ArduinoOutStream cout(Serial);
  const int line_buffer_size = MAX_LINE_LENGTH+1;
  char buffer[line_buffer_size];
  ifstream sdin(CONFIG_FILE_NAME);
  int line_number = 0;

  while (sdin.getline(buffer, line_buffer_size, '\n') || sdin.gcount()) {
    int count = sdin.gcount();
    if (sdin.fail()) {
      //cout << "Partial long line";
      sdin.clear(sdin.rdstate() & ~ios_base::failbit);
    } else if (sdin.eof()) {
      //cout << "Partial final line";  // sdin.fail() is false
    } else {
      count--;  // Don’t include newline in count
      //cout << "Line " << ++line_number;
      setConfig(buffer);
    }
    //cout << " (" << count << " chars): " << buffer << endl;
  }
}


/** **********************************************************************
 * Take a config line, try to split it then apply value.
 */
void setConfig(char *rowBuf) {
  const char s[2] = "=";
  char *key;
  //char *val;
  //get the first token
  key = strtok(rowBuf, s);
  //get the second token
  //val = strtok(NULL, s);
  //Using String because M0 returns incorrect sizeof for arrays not defined at compile time...
  String val = String(strtok(NULL, s));
  val.trim();
  // There's probably a better way than if ... if ... if, but we only do
  // this once, so frankly, my dear...
  if ( strcmp(key, CONFIG_SONAR_OFFSET) == 0 ) {
    if (val && isValidNumber(val) ) {
      //sonarOffset = constrain(atoi(val),0,200);
      sonarOffset = constrain(val.toInt(),0,200);
    }
  } else if ( strcmp(key, CONFIG_TIMEZONE) == 0 ) {
    if (val && isValidNumber(val) ) {
      //Serial.print("setting timezone...  ");
      //timezone = constrain(atof(val),-24,24);
      timezone = constrain(val.toFloat(),-24,24);
      //Format for the iso timestamp
      sprintf(isoTimezone, "%03d:%02d", (int)timezone, (int)(abs(timezone)*60)%60);
      //Overwrite the first position with sign (0 is always +ve)
      isoTimezone[0] = timezone<0?'-':'+';
    }
  } else if ( strcmp(key, CONFIG_SONAR_HEARTBEAT) == 0 ) {
    if (val && isValidNumber(val) ) {
      //sonarHeartbeat = constrain(atoi(val),200,1000);
      sonarHeartbeat = constrain(val.toInt(),200,1000);
    }
  } else if ( strcmp(key, CONFIG_GPS_STALE_LIMIT) == 0 ) {
    if (val && isValidNumber(val) ) {
      gpsStaleLimit = constrain(val.toInt(),2000,60000);
    }
  } else if ( strcmp(key, CONFIG_SD_FLUSH_INTERVAL) == 0 ) {
    if (val && isValidNumber(val) ) {
      //sdFlushInterval = constrain(atoi(val),1000,5000);
      sdFlushInterval = constrain(val.toInt(),1000,5000);
    }
  } else if ( strcmp(key, CONFIG_GPS_LED_PERCENT) == 0 ) {
    if (val && isValidNumber(val) ) {
      //gpsLedPercent = constrain(atoi(val),20,100);
      gpsLedPercent = constrain(val.toInt(),20,100);
    }
  } else if ( strcmp(key, CONFIG_RECORD_LED_PERCENT) == 0 ) {
    //Serial.print("Have CONFIG_RECORD_LED_PERCENT: ");
    //Serial.println(val);
    if (val && isValidNumber(val) ) {
      //Serial.print("Setting CONFIG_RECORD_LED_PERCENT from: ");
      //Serial.print(recordLedPercent);
      //Serial.print(" to: ");
      //recordLedPercent = constrain(atoi(val),20,100);
      recordLedPercent = constrain(val.toInt(),20,100);
      //Serial.println(recordLedPercent);
    }
  }

}

void writeCsvToFile() {
  if (!logFile.isOpen()) {
    char fileName[14];
    //
    sprintf(fileName, "%02d%02d%02d%02d.csv", year()%100, month(), day(), hour());
    //logFile = SD.open(fileName, FILE_WRITE);
    if ( !logFile.open(fileName,O_RDWR | O_CREAT | O_AT_END) ) {
      SD.errorHalt("opening log file for write failed");
      recordState = RECORD_ERROR;
      //Serial.println("Cannot open logFile: " + String(fileName));
    //} else {
      //Serial.println("Opened logFile: " + String(fileName));
    }
    //writeConfigToFile(logFile);
  }
  // Check if we have a logFile (i.e. it opened OK) and can write to it
  if (logFile.isOpen()) { // && (logFile.println(buildCsv()))) {
    char isoDateTime[23];
    // Format the date and time to ISO 8601 standard.
    sprintf(isoDateTime, "%04d-%02d-%02dT%02d:%02d:%02d.%02d%s", year(), month(), day(), hour(), minute(), second(), hundredths(),isoTimezone);
    logFile.print(isoDateTime);
    logFile.print(CSV_DELIMITER);
    logFile.print(gps.location.lng(),6);
    logFile.print(CSV_DELIMITER);
    logFile.print(gps.location.lat(),6);
    logFile.print(CSV_DELIMITER);
    logFile.print(gps.speed.kmph(),0);
    logFile.print(CSV_DELIMITER);
    logFile.print(gps.course.deg(),0);
    logFile.print(CSV_DELIMITER);
    logFile.print(sonarOffset);
    logFile.print(CSV_DELIMITER);
    logFile.print(sonarCm);
    logFile.print(CSV_DELIMITER);
    logFile.print((tagState)>0?1:0);
    logFile.println();
    // Check if it is time to flush writes
    if (loopStart - previousFlush >= sdFlushInterval) {
      previousFlush = loopStart;  // Remember the time
      //Serial.println("Flushing SD...");
      if (!logFile.sync() || logFile.getWriteError() ) {
        
      }
      //logFile.flush();
    }
  } else { // We have failed to either open or write to file.
    recordState = RECORD_ERROR;
  }
}

/**
 * Breaks all the rules of DRY, but it seems Serial makes for loops not increment if
 * passed as reference to common function...
 */
void writeCsvToSerial() {
    char isoDateTime[23];
    // Format the date and time to ISO 8601 standard.
    sprintf(isoDateTime, "%04d-%02d-%02dT%02d:%02d:%02d.%02d%s", year(), month(), day(), hour(), minute(), second(), hundredths(),isoTimezone);
    Serial.print(isoDateTime);
    Serial.print(CSV_DELIMITER);
    Serial.print(gps.location.lng(),6);
    Serial.print(CSV_DELIMITER);
    Serial.print(gps.location.lat(),6);
    Serial.print(CSV_DELIMITER);
    Serial.print(gps.speed.kmph(),0);
    Serial.print(CSV_DELIMITER);
    Serial.print(gps.course.deg(),0);
    Serial.print(CSV_DELIMITER);
    Serial.print(sonarOffset);
    Serial.print(CSV_DELIMITER);
    Serial.print(sonarCm);
    Serial.print(CSV_DELIMITER);
    Serial.print((tagState>0)?1:0);
    Serial.print(CSV_DELIMITER);
    Serial.print(gps.hdop.value());
    Serial.print(CSV_DELIMITER);
    Serial.print(gps.location.age());
    Serial.print(CSV_DELIMITER);
    Serial.print(gps.failedChecksum());
    Serial.print(CSV_DELIMITER);
    Serial.print(SERIAL_BUFFER_SIZE);
    Serial.println();
}

void writeConfig() {
  if ( !configFile.open(CONFIG_FILE_NAME,O_RDWR | O_CREAT | O_AT_END) ) {
      SD.errorHalt("opening config file for write failed");
      recordState = RECORD_ERROR;
  } else {
    //
    configFile.println("Timezone offset from UTC in decimal. Also change for Daylight Savings Time etc");
    configFile.print(CONFIG_TIMEZONE);
    configFile.print(CONFIG_DELIMITER);
    configFile.println(timezone);
    //
    configFile.println("Offset in cm from sonar to elbow.");
    configFile.print(CONFIG_SONAR_OFFSET);
    configFile.print(CONFIG_DELIMITER);
    configFile.println(sonarOffset);
    //
    configFile.println("Milliseconds beween sonar pings. e.g. 250 is 4 times per second");
    configFile.print(CONFIG_SONAR_HEARTBEAT);
    configFile.print(CONFIG_DELIMITER);
    configFile.println(sonarHeartbeat);
    //
    configFile.println("Milliseconds before notifying GPS fix is lost.");
    configFile.print(CONFIG_GPS_STALE_LIMIT);
    configFile.print(CONFIG_DELIMITER);
    configFile.println(gpsStaleLimit);
    //
    configFile.println("Milliseconds between actually writing records to SD card. More frequent will use more battery.");
    configFile.print(CONFIG_SD_FLUSH_INTERVAL);
    configFile.print(CONFIG_DELIMITER);
    configFile.println(sdFlushInterval);
    //
    configFile.println("Percentage brightness of GPS LED. Use to balance output or reduce max.");
    configFile.print(CONFIG_GPS_LED_PERCENT);
    configFile.print(CONFIG_DELIMITER);
    configFile.println(gpsLedPercent);
    //
    configFile.println("Percentage brightness of Record LED. Use to balance output or reduce max.");
    configFile.print(CONFIG_RECORD_LED_PERCENT);
    configFile.print(CONFIG_DELIMITER);
    configFile.println(recordLedPercent);
    //
    configFile.close();
  }
}


/**
 * User provided date time callback function.
 * See SdFile::dateTimeCallback() for usage.
 */
void dateTime(uint16_t* date, uint16_t* time) {
  // User gets date and time from GPS or real-time
  // clock in real callback function

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year(), month(), day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour(), minute(), second());
}

void setLedLevels() {
    //Turn off the near miss led to avoid false lighting
    digitalWrite(NEAR_MISS_LED, LOW);
    delay(10);
    int ldrLevel = analogRead(LDR_PIN);
    //int ledLevel = map(ldrLevel, LDR_MIN, LDR_MAX, LED_MAX, LED_MIN);
    ledLevel = map(ldrLevel, LDR_MIN, LDR_MAX, LED_MIN, LED_MAX);
    ledLevel = constrain(ledLevel, LED_MIN, LED_MAX);
    gpsLedLevel = ledLevel*gpsLedPercent/100;
    recordLedLevel = ledLevel*recordLedPercent/100;
    //Serial.print(ldrLevel);
    //Serial.print(" : ");
    //Serial.print(ledLevel);
    //Serial.print(" : ");
    //Serial.print(gpsLedLevel);
    //Serial.print(" : ");
    //Serial.print(recordLedPercent);
    //Serial.print(" = ");
    //Serial.println(recordLedLevel);
}

/**
 * Flash LED sequences from states
 */
void manageLeds() {
  if ( loopStart - ledTimer > 125 ) { // Eight times per second
    // Store the last time we updated the LEDs
    ledTimer = loopStart;
    if ( ledTimePos == 0 ) {
      setLedLevels();
    }
    // Red LED is for GPS and battery state
    //int r = (bitRead(gpsState | batteryState, (int)ledTimePos/32))*20;
    //Serial.println(r);
    //analogWrite(GPS_LED, r);
    analogWrite(GPS_LED, (bitRead(gpsState | batteryState, (int)ledTimePos/32))?gpsLedLevel:0);
    // Green LED is for recording and tagging
    //Serial.print("recordLed: ");
    //Serial.print((bitRead(gpsState | batteryState, (int)ledTimePos/32)));
    //Serial.print(" * ");
    //Serial.print(recordLedLevel);
    //Serial.print(" = ");
    //Serial.println((bitRead(recordState | tagState, (int)ledTimePos/32))?recordLedLevel:0);
    analogWrite(RECORD_LED, (bitRead(recordState | tagState, (int)ledTimePos/32))?recordLedLevel:0);
    // Blue LED is near miss indicator and test mode.
    digitalWrite(NEAR_MISS_LED, (bitRead(nearMissState | testState, (int)ledTimePos/32)));
    // Move the position along 1/8th of a byte.
    // Because ledTimePos is a byte, it will cycle back to 0 after 8 increments.
    ledTimePos += 32;
  }
}

void checkBatteryVoltage() {
  if ( loopStart - previousVoltageCheck >= VOLTAGE_CHECK_FREQ ) {
      previousVoltageCheck=loopStart;
    // This is for the Adafruit Feather, but calculates millivolts (A9*2*3300)/1024 = 6.445
    // https://learn.adafruit.com/adafruit-feather-32u4-adalogger/power-management
    //Serial.println(analogRead(BATTERY_VOLTAGE_PIN)*6.445);
    if ( (int)(analogRead(BATTERY_VOLTAGE_PIN)*6.445) <= LOW_VOLTAGE_LIMIT ) {
      batteryState = LOW_BATTERY;
    } else {
      batteryState = 0;
    }
  }
}

boolean isValidNumber(String str) {
  boolean isNum=false;
  //Serial.print("Is what: ");
  //Serial.println(str);
  if (!(str[0] == '+' || str[0] == '-' || isDigit(str[0]))) {
    //Serial.print("Returning false: ");
    //Serial.println(str[0]);
    return false;
  }
    //Check remaining chars
  for(byte i=1;i<str.length();i++) {
    //Serial.println(str[i]);
    if (!(isDigit(str[i]) || str[i] == '.')) {
      //Serial.print("Returning false: ");
      //Serial.println(str[i]);
      return false;
    }
  }
  return true;
}

