
#define DEBUG 1 // Comment this line out to disable the debug statements

//Global Variables
float currentTemp = 0;
float currentHumidity = 0;
float currentPressure = 0;

class WeatherDisplay
{
  WeatherDisplay(unsigned long screen);
  void update(void);
}

class WeatherMeasurement
{
  WeatherMeasurement(unsigned long timing);
  void update(void);
}

class WeatherForecast
{
  WeatherMeasurement(unsigned long timing);
  void update(void);
}
