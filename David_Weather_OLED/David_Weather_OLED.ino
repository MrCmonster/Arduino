/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
/*

  PrintTest.pde

  How to use the base class "Print"

  >>> Before compiling: Please remove comment from the constructor of the
  >>> connected graphics display (see below).

  Universal 8bit Graphics Library, https://github.com/olikraus/u8glib/

  Copyright (c) 2012, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

    Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#include "U8glib.h"

// setup u8g object, please remove comment from one of the following constructor calls
// IMPORTANT NOTE: The following list is incomplete. The complete list of supported
// devices with all constructor calls is here: https://github.com/olikraus/u8glib/wiki/device
//U8GLIB_SH1106_128X64 u8g(13, 11, 10, 9);  // SW SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_SH1106_128X64 u8g(4, 5, 6, 7); // SW SPI Com: SCK = 4, MOSI = 5, CS = 6, A0 = 7 (new blue HalTec OLED)
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE); // I2C / TWI
//U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_FAST); // Dev 0, Fast I2C / TWI
//U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NO_ACK); // Display which does not send ACK


#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define DEBUG 1 // Comment this line out to disable the debug statements

//***************************************************************************
// Start of stuff for the BME280
//#define BME_SCK 13
//#define BME_MISO 12
//#define BME_MOSI 11
//#define BME_CS 10

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;
int fontLineSpacing = 0; //default to 0, read later
float currentTemp = 0;
float currentHumidity = 0;
float currentPressure = 0;
// End of stuff for the BME280
//***************************************************************************


//***************************************************************************
// Start of stuff for the forecast
float lastPressure = -1;
float lastTemp = -1;
int lastForecast = -1;

const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];

// this CONVERSION_FACTOR is used to convert from Pa to kPa in forecast algorithm
// get kPa/h by dividing hPa by 10
#define CONVERSION_FACTOR (1.0/10.0)

int minuteCount = 0;
bool firstRound = true;
// average value is used in forecast algorithm.
float pressureAvg;
// average after 2 hours is used as reference value for the next iteration.
float pressureAvg2;

float dP_dt;

// Sleep time between reads (in seconds). Do not change this value as the
// forecast algorithm needs a sample every minute.
const unsigned long SLEEP_TIME = 60000;

const char *weather[] = { "Stable", "Sunny", "Cloudy", "Unstable",
"Thunderstorm", "Unknown-Learning" };

enum FORECAST
{
  STABLE = 0,     // "Stable Weather Pattern"
  SUNNY = 1,      // "Slowly rising Good Weather", "Clear/Sunny "
  CLOUDY = 2,     // "Slowly falling L-Pressure ", "Cloudy/Rain "
  UNSTABLE = 3,   // "Quickly rising H-Press",     "Not Stable"
  THUNDERSTORM = 4, // "Quickly falling L-Press",    "Thunderstorm"
  UNKNOWN = 5     // "Unknown (More Time needed)
};
int forecast = 5;
// End of stuff for the forecast
//**************************************************************************

void setup() {
  Serial.begin(115200);
  Serial.println(F("BME280 initilization"));

  if (! bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    pinMode(LED_BUILTIN, OUTPUT);
    while (1)
    {
      // Blink the LED to indicate an error
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);                       // wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);                       // wait for a second
    }
  }

  /*/
    Serial.println("-- Default Test --");
    Serial.println("normal mode, 16x oversampling for all, filter off,");
    Serial.println("0.5ms standby period");
    delayTime = 10;
    //*/

  // For more details on the following scenarious, see chapter
  // 3.5 "Recommended modes of operation" in the datasheet

  //*
  // weather monitoring
  Serial.println("-- Weather Station Scenario --");
  Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling");
  Serial.println("filter off");
  Serial.println("standby = 10ms");
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF,
                  Adafruit_BME280::STANDBY_MS_10);

  // suggested rate is 1/60Hz (1m)
  // delayTime = 60000;  // in milliseconds
  delayTime = 10; // in milliseconds
  //*/

  /*
      // humidity sensing
      Serial.println("-- Humidity Sensing Scenario --");
      Serial.println("forced mode, 1x temperature / 1x humidity / 0x pressure oversampling");
      Serial.println("= pressure off, filter off");
      bme.setSampling(Adafruit_BME280::MODE_FORCED,
                      Adafruit_BME280::SAMPLING_X1,   // temperature
                      Adafruit_BME280::SAMPLING_NONE, // pressure
                      Adafruit_BME280::SAMPLING_X1,   // humidity
                      Adafruit_BME280::FILTER_OFF );

      // suggested rate is 1Hz (1s)
      delayTime = 1000;  // in milliseconds
  */

  /*
      // indoor navigation
      Serial.println("-- Indoor Navigation Scenario --");
      Serial.println("normal mode, 16x pressure / 2x temperature / 1x humidity oversampling,");
      Serial.println("0.5ms standby period, filter 16x");
      bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                      Adafruit_BME280::SAMPLING_X2,  // temperature
                      Adafruit_BME280::SAMPLING_X16, // pressure
                      Adafruit_BME280::SAMPLING_X1,  // humidity
                      Adafruit_BME280::FILTER_X16,
                      Adafruit_BME280::STANDBY_MS_0_5 );

      // suggested rate is 25Hz
      // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5) + (2 * H_ovs + 0.5)
      // T_ovs = 2
      // P_ovs = 16
      // H_ovs = 1
      // = 40ms (25Hz)
      // with standby time that should really be 24.16913... Hz
      delayTime = 41;

      /*
      // gaming
      Serial.println("-- Gaming Scenario --");
      Serial.println("normal mode, 4x pressure / 1x temperature / 0x humidity oversampling,");
      Serial.println("= humidity off, 0.5ms standby period, filter 16x");
      bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                      Adafruit_BME280::SAMPLING_X1,   // temperature
                      Adafruit_BME280::SAMPLING_X4,   // pressure
                      Adafruit_BME280::SAMPLING_NONE, // humidity
                      Adafruit_BME280::FILTER_X16,
                      Adafruit_BME280::STANDBY_MS_0_5 );

      // Suggested rate is 83Hz
      // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5)
      // T_ovs = 1
      // P_ovs = 4
      // = 11.5ms + 0.5ms standby
      delayTime = 12;
  */


  u8g.setFont(u8g_font_unifont);
  fontLineSpacing = u8g.getFontLineSpacing() + 1; // Add 1 pixel to take the º
  //symbol into account, it's one pixel higher than regular charaters

  Serial.print("fontLineSpacing = ");
  Serial.println(fontLineSpacing, DEC);
  Serial.println("Finished setup()");
}

int counter = 10; // Start at 10 so that the first pass pulls data from
// the sensors instead ofusing the initial values from the code.


void loop() {
  // Setup the timers
  static VirtualDelay screenRedrawDelay, measurementDelay, forecastDelay;
  DO_ONCE
  ( Serial.println("DO_ONCE 1");
    screenRedrawDelay.start(screenRedraw_ms); // start one-shot screenRedrawDelay
    measurementDelay.start(measurement_ms); // start one-shot measurementDelay
    forecastDelay.start(forecast_ms); // start one-shot forecastDelay
    #ifdef DEBUG
      Serial.print("millis=");
      Serial.println(millis());
    #endif
  )

  // if it's time to run the forecast
  if(forecastDelay.elapsed())
    { // update the forecast
      // Reset the timer at the beginnin of the function so we don't add in the
      // time it takes to run the function.
      forecastDelay.start(forecast_ms);
      #ifdef DEBUG
        Serial.print("forecast millis=");
        Serial.println(millis());
      #endif
      forecast = sample(currentPressure);
      // restart the timer
    };

  // if it's time to update the sensor readings
  if(measurementDelay.elapsed())
    { // read the sensors
      // Reset the timer at the beginnin of the function so we don't add in the
      // time it takes to run the function.
      measurementDelay.start(measurement_ms);
      #ifdef DEBUG
        Serial.print("measurement millis=");
        Serial.println(millis());
      #endif
      // Only needed in forced mode! In normal mode, you can remove the next line.
      bme.takeForcedMeasurement(); // has no effect in normal mode
      getValues();
      // restart the timer
    };

  // if it's time to redraw the screen
  if(screenRedrawDelay.elapsed())
    { // redraw the screen frequently so it looks smooth when changing values.
      // Reset the timer at the beginnin of the function so we don't add in the
      // time it takes to run the function.
      screenRedrawDelay.start(screenRedraw_ms);
      u8g.firstPage();
      do {
        printValues(); // Update the screen every time for better refresh rate
      } while ( u8g.nextPage() );
      // restart the timer
    };
}


void printValues() {
  // graphic commands to redraw the complete screen should be placed here

  //  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(0, fontLineSpacing); // Line 1
  // call procedure from base class, http://arduino.cc/en/Serial/Print
  u8g.print("Temp:  ");
  u8g.print(currentTemp, 1);
  u8g.print("\260F"); // '\260' is the ASCII degree symbol

  u8g.setPrintPos(0, fontLineSpacing * 2); // Line 2
  u8g.print("Press: ");
  u8g.print(currentPressure, 1);
  u8g.print(" hPa");

  u8g.setPrintPos(0, fontLineSpacing * 3); // Line 3
  u8g.print("Hum:   ");
  u8g.print(currentHumidity, 1);
  u8g.print("%");

  u8g.setPrintPos(0, fontLineSpacing * 4); // Line 4
  u8g.print(weather[forecast]);
}

void getValues() {
  currentTemp = bme.readTemperature() * 9.0 / 5.0 + 32; // Convert C to F
  currentPressure = bme.readPressure() / 100.0F; // Convert to hPa == milliBar
  currentHumidity = bme.readHumidity();
}

// Algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure)
{
  // Calculate the average of the last n minutes.
  int index = minuteCount % LAST_SAMPLES_COUNT;
  lastPressureSamples[index] = pressure;

  minuteCount++;
  if (minuteCount > 185)
  {
    minuteCount = 6;
  }

  if (minuteCount == 5)
  {
    pressureAvg = getLastPressureSamplesAverage();
  }
  else if (minuteCount == 35)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change * 2; // note this is for t = 0.5hour
    }
    else
    {
      dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
    }
  }
  else if (minuteCount == 65)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) //first time initial 3 hour
    {
      dP_dt = change; //note this is for t = 1 hour
    }
    else
    {
      dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 95)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 1.5; // note this is for t = 1.5 hour
    }
    else
    {
      dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 125)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    pressureAvg2 = lastPressureAvg; // store for later use.
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2; // note this is for t = 2 hour
    }
    else
    {
      dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 155)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2.5; // note this is for t = 2.5 hour
    }
    else
    {
      dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 185)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 3; // note this is for t = 3 hour
    }
    else
    {
      dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
    }
    pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

  int forecast = UNKNOWN;
  if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
  {
    forecast = UNKNOWN;
  }
  else if (dP_dt < (-0.25))
  {
    forecast = THUNDERSTORM;
  }
  else if (dP_dt > 0.25)
  {
    forecast = UNSTABLE;
  }
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
  {
    forecast = CLOUDY;
  }
  else if ((dP_dt > 0.05) && (dP_dt < 0.25))
  {
    forecast = SUNNY;
  }
  else if ((dP_dt > (-0.05)) && (dP_dt < 0.05))
  {
    forecast = STABLE;
  }
  else
  {
    forecast = UNKNOWN;
  }

#ifdef DEBUG
  Serial.print(F("Forecast at minute "));
  Serial.print(minuteCount);
  Serial.print(F(" dP/dt = "));
  Serial.print(dP_dt);
  Serial.print(F("kPa/h --> "));
  Serial.println(weather[forecast]);
#endif

  return forecast;
}

float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
  {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }
  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  return lastPressureSamplesAverage;
}
