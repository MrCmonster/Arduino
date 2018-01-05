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
#include "WeatherClass.h"

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
#include <avdweb_VirtualDelay.h> // non blocking timer


#define DEBUG 1 // Comment this line out to disable the debug statements


void setup() {
  Serial.begin(115200);



}

//int counter = 10; // Start at 10 so that the first pass pulls data from
// the sensors instead ofusing the initial values from the code.

static unsigned long screenRedraw_ms = 11; // use primes so they don't overlap
static unsigned long measurement_ms = 997; // use primes so they don't overlap
static unsigned long forecast_ms = 60000; // needs to be at 60 sec for the
                                          // forecast algorithm

WeatherDisplay      display (screenRedraw_ms);
WeatherMeasurement  measurement(measurement_ms);
WeatherForecast     forecast(forecast_ms);

void loop()
{
  display.update();
  measurement.update();
  forecast.update();






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

  if(screenRedrawDelay.elapsed())
    { // redraw the screen frequently so it looks smooth when changing values.
      u8g.firstPage();
      do {
        printValues(); // Update the screen every time for better refresh rate
      } while ( u8g.nextPage() );
      // restart the timer
      screenRedrawDelay.start(screenRedraw_ms);
    };
  if(measurementDelay.elapsed())
    { // read the sensors
      #ifdef DEBUG
        Serial.print("measurement millis=");
        Serial.println(millis());
      #endif
      // Only needed in forced mode! In normal mode, you can remove the next line.
      bme.takeForcedMeasurement(); // has no effect in normal mode
      getValues();
      // restart the timer
      measurementDelay.start(measurement_ms);
    };
  if(forecastDelay.elapsed())
    { // update the forecast
      #ifdef DEBUG
        Serial.print("forecast millis=");
        Serial.println(millis());
      #endif
      forecast = sample(currentPressure);
      // restart the timer
      forecastDelay.start(forecast_ms);
    };
/*/
  // picture loop for OLED
  u8g.firstPage();
  do {
    if (counter % 10 == 0) // Only pull data on every 10th pass
    {
      getValues();
    }
    if (counter == 235) // Works out to be roughly once a minute. Must be a better way to figure this out other than trial and error.
    {
      counter = 0;
      forecast = sample(currentPressure);
    }
    printValues(); // Update the screen every time for better refresh rate
//    delay(delayTime);
  } while ( u8g.nextPage() );

  counter++;
//*/
}
