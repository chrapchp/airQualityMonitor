/*
From AirGradient author
----------------------------
AirGradient DIY Air Quality Sensor with an ESP8266 Microcontroller.

For build instructions please visit https://www.airgradient.com/diy/

Compatible with the following sensors:
Plantower PMS5003 (Fine Particle Sensor)
SenseAir S8 (CO2 Sensor)
SHT30/31 (Temperature/Humidity Sensor)
----------------------------

 *  @file    sensair8.h
 *  @author  peter c
 *  @date    11/24/2021
 *  @version 0.1
 *  @section Software uses Adafruit libraries and custom sensair8 library
             Compiled with platformIO
             required libraries and board can be found in platformio.ini
             pushes data to a node-red endpoint from which it sends to 
*/

#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <PubSubClient.h>

#include <ArduinoJson.h>

#include <Wire.h>
#include <Streaming.h>
#include <SoftwareSerial.h>

#include <SSD1306Wire.h>
#include <OLEDDisplayUi.h>

#include <Adafruit_SHT31.h>

#include "AQI.h"
#include "sensair8.h"
#include "DA_NonBlockingDelay.h"
#include "PMS.h"
//#include "PM25.h"

#define CO2_RH_POLL_RATE 10000            // ms
#define MQTT_PUBLISH_RATE 30000            // ms
#define FAULT_WAIT_WIFI_CONNECT_RATE 1000 //ms
#define MQTT_FAULT_RATE 500               // ms
#define WIFI_MAX_RETRIES 3
#define WIFI_WAIT_TIME 5000 // ms how long to wait for a connection

#define OLED_REFRESH_RATE 4000 //ms

#define PM25_SLEEP_DURATION 10000 // ms
#define PM25_ACTIVE_RATE 5000     // ms

#define WIFI_FAIL_LED D7

#define SENSOR8_WARMUP_DELAY 10 //

typedef void (*function_t)(SSD1306Wire *display); // for array if functions. OLED display frame

#define TEST "purple2.4"

const char *ssid = WIFI_SSID;
const char *password = WIFI_PWD;
const char *mqtt_server = "192.168.1.86";

// String clientID = "AirQuality002";
// const char *hostCommandTopic = "AirQuality002";
// const char *mqttTopic = "Home/Hydroponic/AirQuality002/EU";
// const char *wifiHostName = "AirQuality002";

String clientID = "AirQuality001";
const char *hostCommandTopic = "AirQuality001";
const char *mqttTopic = "Home/Hydroponic/AirQuality001/EU";
const char *wifiHostName = "AirQuality001";



char mqttMsgOut[350];
WiFiClient wifiClient;
PubSubClient mqttClient(mqtt_server, 1883, wifiClient);
StaticJsonDocument<100> mqttMsgIn;

// forware declarations
void displaySplash(SSD1306Wire *display, uint16 remainingTime);
void displayWifi(SSD1306Wire *display, char status);

void provisionWifi();
void tracePM25Data();
void displayOverlay(SSD1306Wire *display);
void displayHome(SSD1306Wire *display);
void displayPM25(SSD1306Wire *display);
void displayPM25Count1(SSD1306Wire *display);
void displayPM25Count2(SSD1306Wire *display);
void onreadT_RH_CO2Tmr();

void onpublishDataTmr();
void reconnect();
void onMQTTMessage(char *topic, byte *payload, unsigned int length);

void onPM25ReadReady();

void onfaultLEDTmrBlink();
void disableLED();

void onOLEDRefresh();

Adafruit_SHT31 sht31 = Adafruit_SHT31();
float ambientTemperature;
float ambientRelativeHumidity;

SoftwareSerial pmSerial(D5, D6);
#ifdef DEBUG
PMS pm25Sensor(pmSerial, onPM25ReadReady, Serial);
#else
PMS pm25Sensor(pmSerial, onPM25ReadReady);
#endif
PMS::DATA pm25Data;

SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_64_48);

// frames are the single views that slide in
function_t frames[] = {displayHome, displayPM25, displayPM25Count1, displayPM25Count2};
int frameCount = 4;
int currentFrame = 1;

SoftwareSerial co2sensor_serial(D4, D3);
//Sensair8 co2Sensor = Sensair8(&co2sensor_serial, &Serial);
Sensair8 co2Sensor = Sensair8(&co2sensor_serial);
int16_t co2Level;

AQI aqi = AQI();
AQI_Data airQualityData;

const char *aqiUS[7] = {"Good", "UnHealthySG", "Unhealthy", "V-Unhealthy", "Hazardous", "Uknown"};

DA_NonBlockingDelay readT_RH_CO2Tmr = DA_NonBlockingDelay(CO2_RH_POLL_RATE, onreadT_RH_CO2Tmr);
DA_NonBlockingDelay publishDataTmr = DA_NonBlockingDelay(MQTT_PUBLISH_RATE, onpublishDataTmr);
DA_NonBlockingDelay faultLEDTmr = DA_NonBlockingDelay(FAULT_WAIT_WIFI_CONNECT_RATE, onfaultLEDTmrBlink);
DA_NonBlockingDelay refreshDisplayTmr = DA_NonBlockingDelay(OLED_REFRESH_RATE, onOLEDRefresh);

bool wifiStatus = false;

// set to true if you want to connect to wifi. The display will show values only when the sensor has wifi connection
boolean connectWIFI = false;

// change if you want to send the pm25Data to another server
//String APIROOT = "http://hw.airgradient.com/";
String APIROOT = "http://fakeapi.jsonparseronline.com/";

void setup()
{
#ifdef DEBUG
  Serial.begin(9600);
#endif
  sht31.begin(0x44);

  pmSerial.begin(9600);

  display.init();
  display.flipScreenVertically();

  pm25Sensor.passiveMode();
  pm25Sensor.wakeUp();
  // overide sleep duration if required but laser/fan should be allowe some time to ramp up 30s is good low end number
  pm25Sensor.setSleepDuration(10000);

  pinMode(WIFI_FAIL_LED, OUTPUT);
  disableLED();

#ifdef ENABLE_WIFI
  provisionWifi();
  mqttClient.setBufferSize(380);
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(onMQTTMessage);

#endif

  for (int i = SENSOR8_WARMUP_DELAY; i > 0; i--)
  {
    displaySplash(&display, i);
    delay(1000);
  }
}

void loop()
{
  readT_RH_CO2Tmr.refresh();
  faultLEDTmr.refresh();
  pm25Sensor.refresh();
  refreshDisplayTmr.refresh();
#ifdef ENABLE_WIFI
  publishDataTmr.refresh();
  mqttClient.loop();
#endif
}

void reconnect()
{

  if (!mqttClient.connected())
  {
#ifdef DEBUG
    Serial << "Attempting MQTT connection..." << endl;
#endif
    // Create a random client ID
    clientID += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientID.c_str()))
    {
#ifdef DEBUG
      Serial << "Connected.." << endl;
#endif
      //mqttClient.publish("outTopic", "hello world");
      mqttClient.subscribe(hostCommandTopic);
    }
    else
    {
#ifdef DEBUG
      Serial << "MQTT Connect failed:(" << mqttClient.state() << ")" << endl;
#endif

      faultLEDTmr.setDelay(MQTT_FAULT_RATE);
      faultLEDTmr.resume();
    }
  }
}

void provisionWifi()
{
  delay(10);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  bool toggleStatus = false;

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    displayWifi(&display, toggleStatus ? '+' : '-');
    toggleStatus = !toggleStatus;

#ifdef DEBUG
    Serial << ".";
#endif
  }
  randomSeed(micros());
  WiFi.hostname(wifiHostName);
#ifdef DEBUG
  Serial << endl
         << "IP Address:" << WiFi.localIP() << endl;
#endif
}

void displayWifi(SSD1306Wire *display, char status)
{
  char bannerBuff[14];
  display->clear();
  display->setFont(ArialMT_Plain_16);
  display->setTextAlignment(TEXT_ALIGN_LEFT);

  display->drawString(13, 0, "Wifi");

  display->setFont(ArialMT_Plain_10);

  display->drawString(3, 20, "Connecting");

  sprintf(bannerBuff, "%c", status);
  display->drawString(28, 28, bannerBuff);

  display->display();
}

void displaySplash(SSD1306Wire *display, uint16 remainingTime)
{
  char bannerBuff[14];
  display->clear();
  display->setFont(ArialMT_Plain_16);
  display->setTextAlignment(TEXT_ALIGN_LEFT);

  display->drawString(5, 0, "AirMon");

  display->setFont(ArialMT_Plain_10);

  display->drawString(10, 20, "Warm-up");

  sprintf(bannerBuff, "%d", remainingTime);
  display->drawString(28, 28, bannerBuff);

  display->display();
}

void displayOverlay(SSD1306Wire *display)
{
  //char bannerBuff[20];
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);

  display->drawString(0, 0, aqiUS[airQualityData.category]);
}

void displayHome(SSD1306Wire *display)
{

  char bannerBuff[20];
  display->clear();
  displayOverlay(display);
  display->setFont(ArialMT_Plain_10);
  display->setTextAlignment(TEXT_ALIGN_LEFT);

  sprintf(bannerBuff, "CO2: %d", co2Level);
  display->drawString(0, 8, bannerBuff);

  sprintf(bannerBuff, "T: %.1f", ambientTemperature);
  display->drawString(15, 17, bannerBuff);

  sprintf(bannerBuff, "H: %.1f", ambientRelativeHumidity);
  display->drawString(15, 26, bannerBuff);
  display->display();
}

void displayPM25(SSD1306Wire *display)
{
  char bannerBuff[20];
  display->clear();
  displayOverlay(display);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);

  sprintf(bannerBuff, "PM1.0:%d", pm25Data.PM_SP_UG_1_0);
  display->drawString(0, 8, bannerBuff);

  sprintf(bannerBuff, "PM2.5:%d", pm25Data.PM_SP_UG_2_5);
  display->drawString(0, 17, bannerBuff);

  sprintf(bannerBuff, "PM10:%d", pm25Data.PM_SP_UG_10_0);
  display->drawString(4, 26, bannerBuff);
  display->display();
}

void displayPM25Count1(SSD1306Wire *display)
{
  char bannerBuff[20];
  display->clear();
  displayOverlay(display);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);

  sprintf(bannerBuff, ">0.3um: %d", pm25Data.PM_TOTALPARTICLES_0_3);
  display->drawString(0, 8, bannerBuff);

  sprintf(bannerBuff, ">0.5um: %d", pm25Data.PM_TOTALPARTICLES_0_5);
  display->drawString(0, 16, bannerBuff);

  sprintf(bannerBuff, ">1.0um: %d", pm25Data.PM_TOTALPARTICLES_1_0);
  display->drawString(0, 24, bannerBuff);
  display->display();
}

void displayPM25Count2(SSD1306Wire *display)
{
  char bannerBuff[20];
  display->clear();
  displayOverlay(display);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);

  sprintf(bannerBuff, ">2.5um: %d", pm25Data.PM_TOTALPARTICLES_2_5);
  display->drawString(0, 8, bannerBuff);

  sprintf(bannerBuff, ">5.0um: %d", pm25Data.PM_TOTALPARTICLES_5_0);
  display->drawString(0, 16, bannerBuff);

  sprintf(bannerBuff, ">10.um: %d", pm25Data.PM_TOTALPARTICLES_10_0);
  display->drawString(0, 24, bannerBuff);
  display->display();
}

void onPM25ReadReady()
{

  airQualityData.category = UNKNOWN;
  airQualityData.index = -1;

  if (!pm25Sensor.readUntil(pm25Data))
  {
#ifdef DEBUG
    Serial << "Could not read from PM25 sensor" << endl;
#endif
  }
  else
  {
    airQualityData = aqi.get_AQI(pm25Data.PM_SP_UG_1_0, pm25Data.PM_SP_UG_2_5);
  }

#ifdef DEBUG
  tracePM25Data();
#endif
}

void onreadT_RH_CO2Tmr()
{
  co2Level = co2Sensor.getCO2(3);
  ambientTemperature = sht31.readTemperature();
  ambientRelativeHumidity = sht31.readHumidity();

#ifdef DEBUG

  Serial << "CO2 Sensair:" << co2Level << " ppm" << endl;
  Serial << "Temperature:" << ambientTemperature << "C";
  Serial << "Relative Humidity:" << ambientRelativeHumidity << "%" << endl;

#endif
}

void onMQTTMessage(char *topic, byte *payload, unsigned int length)
{
#ifdef DEBUG
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

#endif

  DeserializationError err = deserializeJson(mqttMsgIn, payload);
  if (err)
  {
    Serial << F("failed json read ") << err.f_str() << endl;
  }

  int coms = mqttMsgIn["YI-001"];

  if (coms)
  {
    publishDataTmr.pause();
#ifdef DEBUG
    Serial << "turn OFF MQTT Publish" << endl;
#endif
  }
  else
  {
    publishDataTmr.resume();
#ifdef DEBUG
    Serial << "turn ON MQTT Publish" << endl;
#endif
  }

  //   for (int i = 0; i < length; i++)
  //   {
  //     Serial.print((char)payload[i]);
  //   }
  //   Serial.println();
}

void onpublishDataTmr()
{

  if (!mqttClient.connected())
  {
    reconnect();
  }
  else
  {
    disableLED();
  }

  sprintf(mqttMsgOut,
          "{\"AT-001A\": %d, \"AT-001B\":%d, \"AT-001C\":%d, \"AT-001D\":%d,\"AT-001E\": %d, \"AT-001F\":%d,"
          "\"AT-002A\":%d, \"AT-002B\":%d, \"AT-002C\": %d,"
          "\"AT-003A\":%d, \"AT-003B\":%d, \"AT-003C\":%d,"
          "\"AT-004\":%d, \"AT-005\":%.1f, \"AQI-001A\":%.1f,"
          "\"AQI-001B\":%d, \"TI-001\":%.1f }",
          pm25Data.PM_TOTALPARTICLES_0_3, pm25Data.PM_TOTALPARTICLES_0_5, pm25Data.PM_TOTALPARTICLES_1_0, pm25Data.PM_TOTALPARTICLES_2_5, pm25Data.PM_TOTALPARTICLES_5_0, pm25Data.PM_TOTALPARTICLES_10_0,
          pm25Data.PM_SP_UG_1_0, pm25Data.PM_SP_UG_2_5, pm25Data.PM_SP_UG_10_0,
          pm25Data.PM_AE_UG_1_0, pm25Data.PM_AE_UG_2_5, pm25Data.PM_AE_UG_10_0,
          co2Level, ambientRelativeHumidity, airQualityData.index,
          airQualityData.category, ambientTemperature);

#ifdef DEBUG
  Serial << "MQTT Topic:" << mqttTopic << endl;
  Serial << "Msg:";
  //Serial << "MQTT Message:" << mqttMsgOut << endl;
  Serial.println(mqttMsgOut);
#endif

  mqttClient.publish(mqttTopic, mqttMsgOut);
}

void onfaultLEDTmrBlink()
{
#ifdef DEBUG
  Serial << "Blink" << endl;
#endif
  digitalWrite(WIFI_FAIL_LED, !digitalRead(WIFI_FAIL_LED));
}

void disableLED()
{
  faultLEDTmr.pause();
  digitalWrite(WIFI_FAIL_LED, LOW);
}

void onOLEDRefresh()
{

  frames[currentFrame - 1](&display);
  currentFrame++;
  if (currentFrame > frameCount)
    currentFrame = 1;
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

  Serial << "AQI Index:" << airQualityData.index << " Category:" << airQualityData.category << " name:" << aqiUS[airQualityData.category] << endl;
}
