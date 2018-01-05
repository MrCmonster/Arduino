/***************************************************************************
Class to handle the weather prediction and data display

***************************************************************************/

#include "WeatherClass.h"


class WeatherDisplay
{
  // Class Member Variables
  int fontLineSpacing;
  unsigned long lastUpdate; // will store last time OLED was updated
  unsigned long screenRedrawTime;

  public:

    // Constructor
    WeatherDisplay(unsigned long screen)
    {
      screenRedrawTime = screen;

      u8g.setFont(u8g_font_unifont);
      fontLineSpacing = u8g.getFontLineSpacing() + 1; // Add 1 pixel to take the º
      //symbol into account, it's one pixel higher than regular charaters

      Serial.print("fontLineSpacing = ");
      Serial.println(fontLineSpacing, DEC);
      Serial.println("Finished setup()");

    }

  void update()
  {
    if((millis() - lastUpdate) > screenRedrawTime)  // time to update
    {
      lastUpdate = millis();

      u8g.firstPage();
      do
      {
        // graphic commands to redraw the complete screen should be placed here

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
      while ( u8g.nextPage() );

    }
  }
}

class WeatherMeasurement
{
  //***************************************************************************
  // Start of stuff for the BME280
  //#define BME_SCK 13
  //#define BME_MISO 12
  //#define BME_MOSI 11
  //#define BME_CS 10

  Adafruit_BME280 bme; // I2C
  //Adafruit_BME280 bme(BME_CS); // hardware SPI
  //Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

//  unsigned long delayTime;
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
// delayTime = 10; // in milliseconds
//*/

  // Class Member Variables
  unsigned long int measurementDelay;
  unsigned long lastUpdate; // will store last time measurements were updated

  public:

  // End of stuff for the BME280
  //***************************************************************************

  WeatherMeasurement(unsigned long timing)
  {
    measurementDelay = timing;

    Serial.println(F("BME280 initilization"));

    if (! bme.begin())
    {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      pinMode(LED_BUILTIN, OUTPUT);
      while (1)
      {
        // Blink the LED to indicate an error
        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(1000);                       // wait for a second
        digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
        delay(1000);                       // wait for a second
        // no point continuing, just loop forever
      }
    }
  }

  void update()
  {
    if((millis() - lastUpdate) > forecastDelay)  // time to update
    {
      lastUpdate = millis();

      currentTemp = bme.readTemperature() * 9.0 / 5.0 + 32; // Convert C to F
      currentPressure = bme.readPressure() / 100.0F; // Convert to hPa == milliBar
      currentHumidity = bme.readHumidity();
    }
  }
}

class WeatherForecast
{
  // Class Member Variables
  unsigned long int forecastDelay;
  unsigned long lastUpdate; // will store last time forecast was updated
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

public:

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

// Insert constrouctor here

  // Algorithm found here
  // http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
  // Pressure in hPa -->  forecast done by calculating kPa/h
  int update(float pressure)
  {
    if((millis() - lastUpdate) > forecastDelay)  // time to update
    {
      lastUpdate = millis();

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
}
