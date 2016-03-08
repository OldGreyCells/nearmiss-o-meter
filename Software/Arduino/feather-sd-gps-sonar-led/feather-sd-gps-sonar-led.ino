
/** *************************************************************************** 
 *  Arduino code for the nearmiss-o-meter. More information here:
 *  
 *  http://oldgreycells.blogspot.com/2016/03/near-miss-o-meter-arduino-code.html
 *  
 *  Written for an Adafruit Feather Adalogger but this sketch should be adaptable
 *  to other Arduinos and sonars.
 *  
 *  Uses the NewPing library for 'pulse output' sonar with the Maxsonar (which
 *  runs at 3.3volts). The code will also work with HC-SR04 sonars, but they run
 *  at 5 volts, so extra work will be needed on the circuit (or a 5v Arduino).
 *  
 *  Will also run serial sonars (at 3.3volts with the Feather) by including 
 *  the SerialPing and SoftwareSerial libraries. Comment in/out the appropriate
 *  code below.
 *  
 *  HISTORY
 *  2016/03/04 - Initial release.
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
 
// ----------------------------------------------------------------------------
// Arduino standard libraries (installed with IDE)
// ----------------------------------------------------------------------------
// Serial Peripheral Interface library, required by SD
// https://www.arduino.cc/en/Reference/SPI
#include <SPI.h>
//
// SD Card library
// https://www.arduino.cc/en/Reference/SD
#include <SD.h>
//
// Software Serial Library
// Only requred if you don't have Serial1 or are using SerialPing
// https://www.arduino.cc/en/Reference/SoftwareSerial
//#include <SoftwareSerial.h>
//
// ----------------------------------------------------------------------------
// Third party libraries (will need to be downloaded & installed in IDE)
// ----------------------------------------------------------------------------
// GPS library to read NMEA sentences
// http://arduiniana.org/libraries/tinygps/
#include <TinyGPS.h>
//
// -- Use EITHER --
// Pulse output Ultrasonic sonar library
// https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
#include <NewPing.h>
// --OR--
// Serial output Ultrasonic sonar library
// https://github.com/OldGreyCells/SerialPing
//#include <SerialPing.h>
//

// Constants

// Change this if your nearmiss-o-meter is set nearer the centre of the bike.
const byte SONAR_OFFSET = 0; //Distance in cm from sensor to your elbow.

// Fixed pin allocation (for Adafruit Feather). Do not change these.
const byte SD_CS_PIN = 4; // This is allocated by the SD library.
const byte BATTERY_VOLTAGE_PIN = A9; // Used to measure battery voltage.

// Pin allocation. If you've made errors with the wiring up, you can fix it here!
const byte SONAR_TX_PIN = 5; // The trigger pin (or TX for SoftwareSerial)
const byte SONAR_RX_PIN = 6; // The echo ping (or RX for SoftwareSerial)
const byte SWITCH_PIN = 10;
const byte GPS_LED = 11; //red
const byte RECORD_LED = 12; //green
const byte NEAR_MISS_LED = 13; //blue

// Delimiter for the CSV records.
const char CSV_DELIMITER = ',';
// Milliseconds between actually writing records to SD card. More frequent will use more battery.
const unsigned int SD_FLUSH_INTERVAL = 2000;
// Number of milliseconds to keep recording after GPS fix is lost.
const unsigned int GPS_STALE_LIMIT = 5000;
// Milliseconds between sonar pings.
const int SONAR_HEARTBEAT = 200;
// Number of pings required to get the median.
const byte SONAR_MEDIAN_ITERATIONS = 5;
// Max distance (in cm) to read. Greater than this value will record 0.
const int SONAR_MAX_DISTANCE = 200;
// Distance in cm that counts as a near miss. Only used for led indication. Max 255.
const byte NEAR_MISS_CM = 100;
// Number of clear (>NEAR_MISS_CM) pings before led of turned off.
const byte NEAR_MISS_LED_PINGS = 5; 
// Battery voltage limit that will trigger LED warning, in millivolts.
const int LOW_VOLTAGE_LIMIT = 2800; 

// Momentary (press button) switch handling
const byte SWITCH_DEBOUNCE_TIME = 20; //Increase this if tag toggling is reading bounces
const int SWITCH_SHORT_PRESS_MAX = 500; // Also minimum for medium press...
const int SWITCH_LONG_PRESS_MIN = 5000; // and also maximum for medium press.

// Used as status values and for LED flash frequency
const byte GPS_NO_FIX    = B11111111; // Continuous
const byte GPS_HAVE_FIX  = B00000100; // Once per second
const byte RECORD_ERROR  = B11101110; // Long flash, twice per second
const byte RECORDING     = B00000001; // Once per second
const byte TAGGING       = B00010000; // Twice per second (in combination with RECORDING)
const byte NEAR_MISS     = B10101010; // Four times per second
const byte SONAR_TEST    = B10000000; // One short flash per second
const byte LOW_BATTERY   = B01010101; // Four times per second OR'd with GPS_HAVE_FIX

// Variables

// Set at start of each loop for simple state machine.
unsigned long now = 0;
// The last time sonar was pinged.
unsigned long previousPing = 0;
// The last time the SD card was flushed
unsigned long previousFlush = 0;

// GPS variables
long lat, lon;
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long date, time, speed, course;
unsigned long fixAge = TinyGPS::GPS_INVALID_AGE;

// Sonar variables
byte sonarCm = 0;
byte nearmissCounter = 0;

// Switch handling
int switchDownTime = 0;
byte switchState = HIGH;
long switchLoopTime = 0;
boolean switchPressed = false;

// Used for status and for led flashing
byte gpsState = 0;
byte recordState = 0;
byte tagState = 0;
byte nearMissState = 0;
byte batteryState = 0;
byte testState = 0;

// Timers for LED flashing
long ledTimer = 0;
byte ledTimePos = 0;

// SerialPing and/or TinyGPS may need SoftwareSerial
//SoftwareSerial softwareSerial(SONAR_RX_PIN, SONAR_TX_PIN);

// Create a new TinyGPS instance.
TinyGPS gps;

// Create a NewPing instance
NewPing sonar(SONAR_TX_PIN, SONAR_RX_PIN, SONAR_MAX_DISTANCE);
// --OR--
// Create a SerialPing instance
//SerialPing sonar(softwareSerial, SONAR_MAX_DISTANCE);


// Create a new File.
File logFile;

/** *************************************************************************** 
 * The standard Arduino setup function, run once on startup.
 */
void setup() {
  // Begin communication with the USB serial port for the serial monitor
  Serial.begin(115200);
  // Only some boards need this wait loop. Uncomment if you're having trouble reading from the serial monitor.
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}
  // Begin communication with the serial port for the GPS
  // Some Arduino boards may require SoftwareSerial to do this. If you want to
  // use SoftwareSerial for both GPS and sonar see: 
  // https://www.arduino.cc/en/Reference/SoftwareSerialListen
  Serial1.begin(9600);

  // On a board with hardware Serial1 , this can be used for SerialPing
  //softwareSerial.begin(9600);
  
  // Initialise all the pins to their correct mode.
  pinMode(GPS_LED, OUTPUT);
  pinMode(RECORD_LED, OUTPUT);
  pinMode(NEAR_MISS_LED, OUTPUT);
  pinMode(SONAR_TX_PIN, OUTPUT);
  pinMode(SONAR_RX_PIN, INPUT);
  pinMode(SWITCH_PIN, INPUT);
  // Turn all the LEDs off.
  digitalWrite(GPS_LED, LOW);
  digitalWrite(RECORD_LED, LOW);
  digitalWrite(NEAR_MISS_LED, LOW);
  // The momentary switch connects to ground when pressed so use internal resistor to pull up.
  // (saves having a resistor on the circuit board)
  digitalWrite(SWITCH_PIN, HIGH);
  delay(2000); //Give me chance to turn on serial monitor.
  //Serial.print("Checking SD card...");
  // Check to see if an SD card is inserted.
  if (SD.begin(SD_CS_PIN)) {
    // If we have an SD card...
    for (byte i = 0; i < 5; i++) {
      // ... flash the record LED twice.
      digitalWrite(RECORD_LED, i%2 ? HIGH:LOW);
      delay(500); // wait for a half a second
    }
    //Serial.println("Have SD card");
  } else {
    // No SD card, so we can't record.
    recordState = RECORD_ERROR;
  }
  // We don't yet have a GPS fix...
  gpsState = GPS_NO_FIX;
}

/** *************************************************************************** 
 * The standard Arduino loop function. Called continuously.
 */
void loop() {
  // Reset at the start of each loop.
  now = millis();
  // Check to see if the switch is being pressed & take action if it is.
  readSwitchState();
  // Read from the GPS serial port and if data is available, populate GPS fields.
  readGps();
  // Ping the sonar if we're recording+have gps fix OR we're in test mode.
  if ( ( ( recordState == RECORDING    // Normally set by switch (although may be updated if SD card write fails).
         && gpsState == GPS_HAVE_FIX ) // And we have a GPS fix
       || testState == SONAR_TEST      // OR we're in test mode
       )
     && (now - previousPing >= SONAR_HEARTBEAT) // And we haven't done a ping for SONAR_HEARTBEAT milliseconds...
     && !Serial1.available() ) { // Probably superfluous, but don't want to interrupt GPS serial port
    // Ping the sonar
    pingSonar();
    // If we're recording, write the GPS+ping data to a file.
    // If there has been a previous error writing to the SD card, record state will be set to RECORD_ERROR.
    if ( recordState == RECORDING ) {
      // Record the data
      writeToFile();
    } else { // Otherwise push data to serial monitor (primarily for test mode).
      Serial.println(buildCsv());
    }
  }
  // If we're mot recording but have the log file open, then close it.
  if ( recordState != RECORDING && logFile ) {
    logFile.close();
    tagState = 0; //Always turn off tag if not recording
  }
  // Manage the LED flashing sequences based on the state fields.
  if ( now - ledTimer > 125 ) { // Eight times per second
    // Store the last time we updated the LEDs
    ledTimer = now;
    // Red LED is for GPS and battery state
    digitalWrite(GPS_LED, bitRead(gpsState | batteryState, (int)ledTimePos/32));
    // Green LED is for recording and tagging
    digitalWrite(RECORD_LED, bitRead(recordState | tagState, (int)ledTimePos/32));
    // Blue LED is near miss indicator and test mode.
    digitalWrite(NEAR_MISS_LED, bitRead(nearMissState | testState, (int)ledTimePos/32));
    // Move the position along 1/8th of a byte.
    // Because ledTimePos is a byte, it will cycle back to 0 after 8 increments.
    ledTimePos += 32;
    // This is for the Adafruit Feather, but calculates millivolts (A9*2*3300)/1024 = 6.445
    // https://learn.adafruit.com/adafruit-feather-32u4-adalogger/power-management
    // It's in the ledTimer section 'cos we don't need to check it on every loop...
    if ( (int)(analogRead(BATTERY_VOLTAGE_PIN)*6.445) <= LOW_VOLTAGE_LIMIT ) {
      batteryState = LOW_BATTERY;
    } else {
      batteryState = 0;
    }
  }  
} // End of loop()

/** *************************************************************************** 
 * Read median distance with the ultrasonic sonar.
 */
void pingSonar() {
  // Remember the time of this ping
  previousPing = now;
  // Get the median microseconds and converto to cm.
  sonarCm = sonar.convert_cm(sonar.ping_median(SONAR_MEDIAN_ITERATIONS));
  // If sonarCm is not 0, deduct the SONAR_OFFSET.
  sonarCm = (sonarCm)?sonarCm-SONAR_OFFSET:0;
  // Flash the blue LED if a near miss occurs.
  // This is also used to determine if maxsonar has barfed.
  if (sonarCm != 0 && sonarCm <= NEAR_MISS_CM ) {
    nearMissState = NEAR_MISS;
    // Reset the counter becuse were still having a near miss.
    nearmissCounter = 0;
  }
  nearmissCounter++;
  // We're in the clear for more than NEAR_MISS_LED_PINGS so...
  if (nearmissCounter > NEAR_MISS_LED_PINGS ) {
    // ...set the state to zero.
    nearMissState = 0;
  }
}

/**
 * Write GPS, distance and tag data to file.
 */
void writeToFile() {
  if (!logFile) {
    char fileName[14];
    //
    sprintf(fileName, "/%02d%02d%02d%02d.csv", year%100, month, day, hour);
    logFile = SD.open(fileName, FILE_WRITE);
    if ( !logFile) {
      recordState = RECORD_ERROR;
      //Serial.println("Cannot open logFile: " + String(fileName));
    //} else {
      //Serial.println("Opened logFile: " + String(fileName));
    }
  }
  // Check if we have a logFile (i.e. it opened OK) and can write to it
  if (logFile && (logFile.println(buildCsv()))) {
    // Check if it is time to flush writes
    if (now - previousFlush >= SD_FLUSH_INTERVAL) {
      previousFlush = now;  // Remember the time
      //Serial.println("Flushing SD...");
      logFile.flush();
    }
  } else { // We have failed to either open or write to file.
    recordState = RECORD_ERROR;
  }
}

/** *************************************************************************** 
 * Read data from GPS try to decode it (using TinyGPS library).
 */
void readGps() {
  // While we have data transmitted from the GPS.
  while (Serial1.available()) {
    // Read from the serial port
    int c = Serial1.read();
    // If we don't have a GPS fix, send data to the serial monitor (for debugging if required)
    if ( gpsState != GPS_HAVE_FIX ) {
      Serial.write(c);
    }
    // TinyGPS::encode() receives one char at a time and encodes a NMEA sentence.
    // returns true if it has a full, decodeable sentence
    if (gps.encode(c)) {
      // Populate the lon, lat and the age of the GPS fix
      gps.get_position(&lat, &lon, &fixAge);
      // Get the data and time from the GPS data.
      gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fixAge);
      // Read the speed (in knots)
      speed = gps.speed();
      // ...and the course (aka heading) in 1/100 of a degree
      course = gps.course();
      // Sometimes (on first fix) TinyGPS will say it has a valid age but does not have valid data.
      // Probably need to build in a short delay before trusting fix.
      if ( (fixAge == TinyGPS::GPS_INVALID_AGE)
         || (fixAge > GPS_STALE_LIMIT) //fixAge is populated by GPS - will not be updated if GPS stops
         || month == 0 //Sometimes TinyDNS returns time but not date...
         || (hour == 0 && minute == 0)) { //Sometimes TinyDNS returns date but not time. TODO this will barf for 1 min after midnight.
        gpsState = GPS_NO_FIX;
      } else {
        gpsState = GPS_HAVE_FIX;
      }
    }
  }
}

/** *************************************************************************** 
 * Build the CSV string and return it to caller.
 * We are not using the float function for lon/lat in TinyGPS, so we have to 
 * format with sprintf. This saves about 2,000 bytes; without it we'd have 
 * to reduce functionality.
 * The logic is as follows:
 * "%0d.%04d" - This is the format. A number with leading zero, decimal point 
 * and then 4 more digits.
 * The two '%'s means sprintf is expecting two arguments:
 * Arg 1: Divide lon or lat by 1,000,000 and strip the sign (i.e. abs).
 * The sign is stripped because sprinf will not put a '-' in front of a 0 for
 * lon/lat close to Greenwich or equator.
 * Arg 2: Divide lon or lat by 100 and mod 10,000, stripping the sign.
 * The sign is stripped because we don't want a sign after the decimal point!
 * And the divide before mod is because sprintf only appears to deal with 
 * ints (max 65,535).
 */
String buildCsv() {
    // A buffer for building the date as a string
    char isoDateTime[25];
    // Format the date and time to ISO 8601 standard.
    sprintf(isoDateTime, "%02d-%02d-%02dT%02d:%02d:%02d.%02dZ", year, month, day, hour, minute, second, hundredths);
    char cLon[10];
    sprintf(cLon, "%0d.%04d", (int)((lon<0?-lon:lon)/1000000), ((int)(((lon<0?-lon:lon)/100)%10000)));
    char cLat[9];
    sprintf(cLat, "%0d.%04d", (int)((lat<0?-lat:lat)/1000000), ((int)(((lat<0?-lat:lat)/100)%10000)));
    return String(isoDateTime)           // Date and time
        + CSV_DELIMITER + (lon<0?"-":"") + String(cLon) // Longitude to 4 decimal places, reinstating sign.
        + CSV_DELIMITER + (lat<0?"-":"") + String(cLat) // Latitude to 4 decimal places, reinstating sign.
        + CSV_DELIMITER + String((int)(speed*0.01852))  // Convert from knots to kph
        + CSV_DELIMITER + String((int)(course/100)) // Divided by 100.
        + CSV_DELIMITER + ((sonarCm)?sonarCm-SONAR_OFFSET:0) // Deduct offset if ping distance is not 0
        + CSV_DELIMITER + (tagState>0)   // Will print 1 (true) if tagging
        ;
}

/** *************************************************************************** 
 * Measure the length of time the switch is pressed and take required action
 * when released. Also acts as a debouncer.
 */
void readSwitchState() {
  if ( now != switchLoopTime ) {
    switchState = digitalRead(SWITCH_PIN);
    if ( switchState == LOW ) { //Switch has been pressed
      // There is a small risk of incremental bounces adding up, but this has not shown up in testing.
      // switchLoopTime is initialised as 0, so deduct 'now' on first loop.
      switchDownTime += now-(switchLoopTime?switchLoopTime:now); 
    }
    if ( switchDownTime > SWITCH_DEBOUNCE_TIME ) {
      switchPressed = true;
      if ( switchDownTime > SWITCH_SHORT_PRESS_MAX 
         && ( ( recordState==0 
            && gpsState==GPS_HAVE_FIX )
            || recordState==RECORDING
            )
        )  {
        digitalWrite(RECORD_LED, HIGH);
      }
      if ( switchDownTime > SWITCH_LONG_PRESS_MIN )  {
        digitalWrite(RECORD_LED, LOW);
        digitalWrite(NEAR_MISS_LED, HIGH);
      }
    }
    if ( switchPressed && switchState == HIGH ) { //Switch has been released
      //Serial.println("Switch pressed for: " + String(ms));
      if ( switchDownTime < SWITCH_SHORT_PRESS_MAX ) { // Short press
        //Serial.println("Short press");
        if ( recordState == RECORDING ) { // Only allow tagging if recording.
          tagState = (tagState==0)?TAGGING:0; // Toggle the tag state
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
    switchLoopTime = now;
  }  
}


