
#include <Streaming.h>
#include <SoftwareSerial.h>
#include "PMS.h"

void onPM25ReadReady();
void tracePM25Data();

SoftwareSerial pmSerial(D5, D6);                   // PM25 sensor wired to D5,D6 on ESP8266 mini
PMS pm25Sensor(pmSerial, onPM25ReadReady, Serial); // default sleep duration is 30s
PMS::DATA pm25Data;

void setup()
{
  Serial.begin(9600);
  pmSerial.begin(9600);
  pm25Sensor.passiveMode();
  pm25Sensor.wakeUp();
  // overide sleep duration if required but laser/fan should be allowe some time to ramp up 30s is good low end number
  pm25Sensor.setSleepDuration(10000);
}

void loop()
{

  pm25Sensor.refresh();
}

// invoked when data is read to be read
void onPM25ReadReady()
{

  if (!pm25Sensor.readUntil(pm25Data)) // default time out is 1s
  //if (!pm25Sensor.readUntil(pm25Data, 2000))  //2 second timeout
  {

    Serial << "Could not read from PM25 sensor" << endl;
    pm25Sensor.sleep(); // sometimes during startup things get mangled, sleep and things will recover.
  }
  else
  {
    tracePM25Data();
  }
}

// from adafruit demo
void tracePM25Data()
{
  Serial.println("PM25 Data");

  Serial.println();
  Serial.println(F("---------------------------------------"));
  Serial.println(F("Concentration Units (standard)"));
  Serial.println(F("---------------------------------------"));
  Serial.print(F("PM 1.0: "));
  Serial.print(pm25Data.PM_SP_UG_1_0);
  Serial.print(F("\t\tPM 2.5: "));
  Serial.print(pm25Data.PM_SP_UG_2_5);
  Serial.print(F("\t\tPM 10: "));
  Serial.println(pm25Data.PM_SP_UG_10_0);
  Serial.println(F("Concentration Units (environmental)"));
  Serial.println(F("---------------------------------------"));
  Serial.print(F("PM 1.0: "));
  Serial.print(pm25Data.PM_AE_UG_1_0);
  Serial.print(F("\t\tPM 2.5: "));
  Serial.print(pm25Data.PM_AE_UG_2_5);
  Serial.print(F("\t\tPM 10: "));
  Serial.println(pm25Data.PM_AE_UG_10_0);
  Serial.println(F("---------------------------------------"));
  Serial.print(F("Particles > 0.3um / 0.1L air:"));
  Serial.println(pm25Data.PM_TOTALPARTICLES_0_3);
  Serial.print(F("Particles > 0.5um / 0.1L air:"));
  Serial.println(pm25Data.PM_TOTALPARTICLES_0_5);
  Serial.print(F("Particles > 1.0um / 0.1L air:"));
  Serial.println(pm25Data.PM_TOTALPARTICLES_1_0);
  Serial.print(F("Particles > 2.5um / 0.1L air:"));
  Serial.println(pm25Data.PM_TOTALPARTICLES_2_5);
  Serial.print(F("Particles > 5.0um / 0.1L air:"));
  Serial.println(pm25Data.PM_TOTALPARTICLES_5_0);
  Serial.print(F("Particles > 10 um / 0.1L air:"));
  Serial.println(pm25Data.PM_TOTALPARTICLES_10_0);
  Serial.println(F("---------------------------------------"));
}
