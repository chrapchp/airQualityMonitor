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
 *  @history 11/25/2024 - integrated with home assistant
 *  @section Software uses Adafruit libraries and custom sensair8 library
             Compiled with platformIO
             required libraries and board can be found in platformio.ini
             pushes data to a node-red endpoint from which it sends to
*/


/*  
  Wrapper around Home Assistant libary to handle common steps for devices
  auto discovery mqtt topic homeassistant/sensor/{uniqueID}/config
  where unique id is deviceID from configPortal or mac if not defined

  data is always published to mqtt
  aha/{uniqueID}

  listens on homeassistant/sensor/{uniqueID}
  Only chekcs enable 
  = 1 enables auto-discovery on home assistant
  = 0 removes devices from home assistant
  {
  "device":
    {
      "name": "",
      "host": "",
      "token": "",
      "mqttIP": "",
      "enable":1
    }
}
  
*/
#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
// #include <PubSubClient.h>

#include <ArduinoJson.h>
#include <ArduinoHA.h>
#include <Wire.h>
#include <Streaming.h>
#include <SoftwareSerial.h>

#include <SSD1306Wire.h>
#include <OLEDDisplayUi.h>

#include <Adafruit_SHT31.h>

#include <DebugLog.h>
#include <DA_NonBlockingDelay.h>
#include <DA_DiscreteInput.h>
#include <HA_Helper.h>

#include "AQI.h"
#include "sensair8.h"

#include "PMS.h"
// #include "PM25.h"

#define CO2_RH_POLL_RATE 10000            // ms
#define FAULT_WAIT_WIFI_CONNECT_RATE 1000 // ms
#define MQTT_FAULT_LED_RATE 500           // ms

// OLED display info
// String clientID = "AirQuality001";
String displayID = "07-Dec-24";

#define OLED_REFRESH_RATE 4000 // ms

#define PM25_SLEEP_DURATION 10000 // ms
#define PM25_ACTIVE_RATE 5000     // ms

#define WIFI_FAIL_LED D7
#define PUSH_BUTTON D8

#define SENSOR8_WARMUP_DELAY 10 //

typedef void (*function_t)(SSD1306Wire *display); // for array if functions. OLED display frame

/* Start - Home Assistant Stuff */
#define SETUP_PIN D1
#define PUBLISH_INTERVAL 30000 // 5 seconds
const char *ORGNAME = "Cogito Methods";
const char *APPVERSION = "1.1.0";
const char *MODEL = "D1-Mini";

WiFiManager wm;

WiFiClient client;

HA_Helper haHelper;
HADevice device;
HAMqtt mqtt(client, device);

// Idle Timer Ev

// uint delemete;
void onIdle();
DA_NonBlockingDelay idleTmr = DA_NonBlockingDelay(PUBLISH_INTERVAL, onIdle);

void onMqttMessage(const char *topic, const uint8_t *payload, uint16_t length);
void onMqttConnected();
void onMqttDisconnected();
void onMqttStateChanged(HAMqtt::ConnectionState state);

JsonDocument jsonDoc;

/* End - Home Assistant Stuff */

/* Unique to Implementation*/

String HAPM0_3SensorID = "AT-001A";
String HAPM0_5SensorID = "AT-001B";
String HAPM1_0SensorID = "AT-001C";
String HAPM2_5SensorID = "AT-001D";
String HAPM5_0SensorID = "AT-001E";
String HAPM10_0SensorID = "AT-001F";

String HA_SP_UG_1_0SensorID = "AT-002A";
String HA_SP_UG_2_5SensorID = "AT-002B";
String HA_SP_UG_10_0SensorID = "AT-002C";

String HA_AE_UG_1_0SensorID = "AT-003A";
String HA_AE_UG_2_5SensorID = "AT-003B";
String HA_AE_UG_10_0SensorID = "AT-003C";

String HACO2SensorID = "AT-004";
String HAHumiditySensorID = "AT-005";

String HAAQI_SensorID = "AQI-001A";
String HACategory_SensorID = "AQI-001B";

String HATemperatureSensorID = "TI-001";

HASensorNumber *HAPM0_3Sensor;
HASensorNumber *HAPM0_5Sensor;
HASensorNumber *HAPM1_0Sensor;
HASensorNumber *HAPM2_5Sensor;
HASensorNumber *HAPM5_0Sensor;
HASensorNumber *HAPM10_0Sensor;

HASensorNumber *HA_SP_UG_1_0Sensor;
HASensorNumber *HA_SP_UG_2_5Sensor;
HASensorNumber *HA_SP_UG_10_0Sensor;

HASensorNumber *HA_AE_UG_1_0Sensor;
HASensorNumber *HA_AE_UG_2_5Sensor;
HASensorNumber *HA_AE_UG_10_0Sensor;

HASensorNumber *HACO2Sensor;
HASensorNumber *HAHumiditySensor;
HASensorNumber *HAAQI_Sensor;
HASensorNumber *HACategory_Sensor;
HASensorNumber *HATemperatureSensor;

void setupHA()
{
  haHelper.referenceDevice(&device);
  haHelper.referenceMqtt(&mqtt);
  haHelper.refenceWifiManager(&wm);
  haHelper.setupUIParams();
}

void setupDeviceSensor()
{
  haHelper.setupDevice(ORGNAME, APPVERSION, MODEL, haHelper.getDeviceName().c_str());

  String id = "-" + haHelper.getToken();

  HAPM0_3SensorID = HAPM0_3SensorID + id;
  HAPM0_5SensorID = HAPM0_5SensorID + id;
  HAPM1_0SensorID = HAPM1_0SensorID + id;

  HAPM2_5SensorID = HAPM2_5SensorID + id;
  HAPM5_0SensorID = HAPM5_0SensorID + id;
  HAPM10_0SensorID = HAPM10_0SensorID + id;

  HA_SP_UG_1_0SensorID = HA_SP_UG_1_0SensorID + id;
  HA_SP_UG_2_5SensorID = HA_SP_UG_2_5SensorID + id;
  HA_SP_UG_10_0SensorID = HA_SP_UG_10_0SensorID + id;

  HA_AE_UG_1_0SensorID = HA_AE_UG_1_0SensorID + id;
  HA_AE_UG_2_5SensorID = HA_AE_UG_2_5SensorID + id;
  HA_AE_UG_10_0SensorID = HA_AE_UG_10_0SensorID + id;
  HACO2SensorID = HACO2SensorID + id;
  HAHumiditySensorID = HAHumiditySensorID + id;
  HAAQI_SensorID = HAAQI_SensorID + id;
  HACategory_SensorID = HACategory_SensorID + id;
  HATemperatureSensorID = HATemperatureSensorID + id;

  LOG_TRACE(id);

  HAPM0_3Sensor = new HASensorNumber(HAPM0_3SensorID.c_str(), HASensorNumber::PrecisionP0);
  HAPM0_5Sensor = new HASensorNumber(HAPM0_5SensorID.c_str(), HASensorNumber::PrecisionP0);
  HAPM1_0Sensor = new HASensorNumber(HAPM1_0SensorID.c_str(), HASensorNumber::PrecisionP0);
  HAPM2_5Sensor = new HASensorNumber(HAPM2_5SensorID.c_str(), HASensorNumber::PrecisionP0);
  HAPM5_0Sensor = new HASensorNumber(HAPM5_0SensorID.c_str(), HASensorNumber::PrecisionP0);
  HAPM10_0Sensor = new HASensorNumber(HAPM10_0SensorID.c_str(), HASensorNumber::PrecisionP0);
  HA_SP_UG_1_0Sensor = new HASensorNumber(HA_SP_UG_1_0SensorID.c_str(), HASensorNumber::PrecisionP0);
  HA_SP_UG_2_5Sensor = new HASensorNumber(HA_SP_UG_2_5SensorID.c_str(), HASensorNumber::PrecisionP0);
  HA_SP_UG_10_0Sensor = new HASensorNumber(HA_SP_UG_10_0SensorID.c_str(), HASensorNumber::PrecisionP0);
  HA_AE_UG_1_0Sensor = new HASensorNumber(HA_AE_UG_1_0SensorID.c_str(), HASensorNumber::PrecisionP0);
  HA_AE_UG_2_5Sensor = new HASensorNumber(HA_AE_UG_2_5SensorID.c_str(), HASensorNumber::PrecisionP0);
  HA_AE_UG_10_0Sensor = new HASensorNumber(HA_AE_UG_10_0SensorID.c_str(), HASensorNumber::PrecisionP0);
  HACO2Sensor = new HASensorNumber(HACO2SensorID.c_str(), HASensorNumber::PrecisionP0);

  HAHumiditySensor = new HASensorNumber(HAHumiditySensorID.c_str(), HASensorNumber::PrecisionP1);
  HAAQI_Sensor = new HASensorNumber(HAAQI_SensorID.c_str(), HASensorNumber::PrecisionP1);
  HACategory_Sensor = new HASensorNumber(HACategory_SensorID.c_str(), HASensorNumber::PrecisionP0);
  HATemperatureSensor = new HASensorNumber(HATemperatureSensorID.c_str(), HASensorNumber::PrecisionP1);

  HACO2Sensor->setIcon("mdi:molecule-co2");
  HACO2Sensor->setName("CO2");
  HACO2Sensor->setUnitOfMeasurement("ppm");

  HAHumiditySensor->setIcon("mdi:water-percent");
  HAHumiditySensor->setName("Humidity");
  HAHumiditySensor->setUnitOfMeasurement("");

  HAAQI_Sensor->setIcon("mdi:vector-point-plus");
  HAAQI_Sensor->setName("AQI-US");
  HAAQI_Sensor->setUnitOfMeasurement("");

  HACategory_Sensor->setIcon("mdi:shape-outline");
  HACategory_Sensor->setName("Category");
  HACategory_Sensor->setUnitOfMeasurement("");

  HATemperatureSensor->setIcon("mdi:thermometer");
  HATemperatureSensor->setName("Temperature");
  HATemperatureSensor->setUnitOfMeasurement("C");

  // HAPM0_3Sensor->setIcon("mdi:thermometer");
  HAPM0_3Sensor->setName("Particles - 0.3");
  HAPM0_3Sensor->setUnitOfMeasurement("um/0.1L Air");

  // HAPM0_5Sensor->setIcon("mdi:thermometer");
  HAPM0_5Sensor->setName("Particles - 0.5");
  HAPM0_5Sensor->setUnitOfMeasurement("um/0.1L Air");

  // HAPM1_0Sensor->setIcon("mdi:thermometer");
  HAPM1_0Sensor->setName("Particles - 1.0");
  HAPM1_0Sensor->setUnitOfMeasurement("um/0.1L Air");

  // HAPM2_5Sensor->setIcon("mdi:thermometer");
  HAPM2_5Sensor->setName("Particles - 2.5");
  HAPM2_5Sensor->setUnitOfMeasurement("um/0.1L Air");

  // HAPM5_0Sensor->setIcon("mdi:thermometer");
  HAPM5_0Sensor->setName("Particles - 5.0");
  HAPM5_0Sensor->setUnitOfMeasurement("um/0.1L Air");

  // HAPM10_0Sensor->setIcon("mdi:thermometer");
  HAPM10_0Sensor->setName("Particles - 10.0");
  HAPM10_0Sensor->setUnitOfMeasurement("um/0.1L Air");

  // HA_SP_UG_1_0Sensor->setIcon("mdi:thermometer");
  HA_SP_UG_1_0Sensor->setName("Sea Level - 1.0");
  HA_SP_UG_1_0Sensor->setUnitOfMeasurement("um/m3");

  // HA_SP_UG_2_5Sensor->setIcon("mdi:thermometer");
  HA_SP_UG_2_5Sensor->setName("Sea Level - 2.5");
  HA_SP_UG_2_5Sensor->setUnitOfMeasurement("um/m3");

  // HA_SP_UG_10_0Sensor->setIcon("mdi:thermometer");
  HA_SP_UG_10_0Sensor->setName("Sea Level - 10.0");
  HA_SP_UG_10_0Sensor->setUnitOfMeasurement("um/m3");

  // HA_AE_UG_1_0Sensor->setIcon("mdi:thermometer");
  HA_AE_UG_1_0Sensor->setName("Ambient - 1.0");
  HA_AE_UG_1_0Sensor->setUnitOfMeasurement("um/m3");

  // HA_AE_UG_2_5Sensor->setIcon("mdi:thermometer");
  HA_AE_UG_2_5Sensor->setName("Ambient - 2.5");
  HA_AE_UG_2_5Sensor->setUnitOfMeasurement("um/m3");

  // HA_AE_UG_10_0Sensor->setIcon("mdi:thermometer");
  HA_AE_UG_10_0Sensor->setName("Ambient - 10.0");
  HA_AE_UG_10_0Sensor->setUnitOfMeasurement("um/m3");

  mqtt.onMessage(onMqttMessage);
  mqtt.onConnected(onMqttConnected);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(60);

  mqtt.begin(haHelper.getMqttIP());
}

// const char *hostCommandTopic = "AirQuality014";
// const char *mqttTopic = "Home/Hydroponic/AirQuality014/EU";
// const char *wifiHostName = "AirQuality014";

// String displayID = "003-Sep21-23";
// String clientID = "AirQuality003";
// const char *hostCommandTopic = "AirQuality003";
// const char *mqttTopic = "Home/Hydroponic/AirQuality003/EU";
// const char *wifiHostName = "AirQuality003";

// String displayID = "002-Sep21-23";
// String clientID = "AirQuality002";
// const char *hostCommandTopic = "AirQuality002";
// const char *mqttTopic = "Home/Hydroponic/AirQuality002/EU";
// const char *wifiHostName = "AirQuality002";

// String displayID = "001-Sep21-23";
//  String clientID = "AirQuality001";
//  const char *hostCommandTopic = "AirQuality001";
//  const char *mqttTopic = "Home/Hydroponic/AirQuality001/EU";
//  const char *wifiHostName = "AirQuality001";

// forward declarations
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

void onPushButton(bool state, int aPin);

void onPM25ReadReady();

void onfaultLEDTmrBlink();
void disableLED();

void onOLEDRefresh();

Adafruit_SHT31 sht31 = Adafruit_SHT31();
float ambientTemperature;
float ambientRelativeHumidity;

SoftwareSerial pmSerial(D5, D6);
#if defined(DBG_LOG) && defined(DBG_SENSOR)
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
// Sensair8 co2Sensor = Sensair8(&co2sensor_serial, &Serial);
Sensair8 co2Sensor = Sensair8(&co2sensor_serial);
int16_t co2Level;

AQI aqi = AQI();
AQI_Data airQualityData;

const char *aqiUS[7] = {"Good", "UnHealthySG", "Unhealthy", "V-Unhealthy", "Hazardous", "Uknown"};

DA_NonBlockingDelay readT_RH_CO2Tmr = DA_NonBlockingDelay(CO2_RH_POLL_RATE, onreadT_RH_CO2Tmr);

DA_NonBlockingDelay faultLEDTmr = DA_NonBlockingDelay(FAULT_WAIT_WIFI_CONNECT_RATE, onfaultLEDTmrBlink);
DA_NonBlockingDelay refreshDisplayTmr = DA_NonBlockingDelay(OLED_REFRESH_RATE, onOLEDRefresh);

DA_DiscreteInput HS_001 = DA_DiscreteInput(PUSH_BUTTON,
                                           DA_DiscreteInput::RisingEdgeDetect,
                                           false);
bool isDisplayEnabled = false;

/**
 * @brief - handle publishing of data to MQTT
 *
 */
void onIdle()
{

  if (haHelper.getConfig()->getCustomValue() == 1)
  {

#ifdef DBG_LOG
    // LOG_ATTACH_SERIAL(Serial);

    LOG_TRACE(String("mqtt IP:" + haHelper.getMqttIP().toString()));
    LOG_TRACE(String("_deviceName:" + haHelper.getDeviceName()));
    LOG_TRACE(String("_token:" + haHelper.getToken()));
    LOG_TRACE(String("_hostName:" + haHelper.getHostName()));
    LOG_TRACE(F("Updating sensor"));
#endif

    HACO2Sensor->setValue(co2Level, true);
    HAHumiditySensor->setValue(ambientRelativeHumidity, true);
    HATemperatureSensor->setValue(ambientTemperature, true);

    HAPM0_3Sensor->setValue(pm25Data.PM_TOTALPARTICLES_0_3, true);
    HAPM0_5Sensor->setValue(pm25Data.PM_TOTALPARTICLES_0_5, true);
    HAPM1_0Sensor->setValue(pm25Data.PM_TOTALPARTICLES_1_0, true);
    HAPM2_5Sensor->setValue(pm25Data.PM_TOTALPARTICLES_2_5, true);
    HAPM5_0Sensor->setValue(pm25Data.PM_TOTALPARTICLES_5_0, true);
    HAPM10_0Sensor->setValue(pm25Data.PM_TOTALPARTICLES_10_0, true);

    HA_SP_UG_1_0Sensor->setValue(pm25Data.PM_SP_UG_1_0, true);
    HA_SP_UG_2_5Sensor->setValue(pm25Data.PM_SP_UG_2_5, true);
    HA_SP_UG_10_0Sensor->setValue(pm25Data.PM_SP_UG_10_0, true);

    HA_AE_UG_1_0Sensor->setValue(pm25Data.PM_AE_UG_1_0, true);
    HA_AE_UG_2_5Sensor->setValue(pm25Data.PM_AE_UG_2_5, true);
    HA_AE_UG_10_0Sensor->setValue(pm25Data.PM_AE_UG_10_0, true);

    HAAQI_Sensor->setValue(airQualityData.index, true);
    HACategory_Sensor->setValue(airQualityData.category, true);

    // HACO2Sensor.setValue(200 + delemete);
    // delemete++;
    //     HAHumiditySensor.setValue(25);
    //     HATemperatureSensor.setValue(20);

    //     HAPM0_3Sensor.setValue(1);
    //     HAPM0_5Sensor.setValue(2);
    //     HAPM1_0Sensor.setValue(3);
    //     HAPM2_5Sensor.setValue(4);
    //     HAPM5_0Sensor.setValue(5);
    //     HAPM10_0Sensor.setValue(6);

    //     HA_SP_UG_1_0Sensor.setValue(7);
    //     HA_SP_UG_2_5Sensor.setValue(8);
    //     HA_SP_UG_10_0Sensor.setValue(9);

    //     HA_AE_UG_1_0Sensor.setValue(10);
    //     HA_AE_UG_2_5Sensor.setValue(11);
    //     HA_AE_UG_10_0Sensor.setValue(12);

    //     HAAQI_Sensor.setValue(13);
    //     HACategory_Sensor.setValue(14);

    //  #if defined(DBG_LOG) && defined(DBG_SENSOR)
    //    tracePM25Data();
    //  #endif
  }
}

void setup()
{
  haHelper.begin();
  haHelper.loadSettings();

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

  // pinMode(SETUP_PIN, INPUT_PULLUP);
#ifdef DBG_LOG
  Serial.begin(115200);
#endif

  if (digitalRead(PUSH_BUTTON) == HIGH)
  {
#ifdef DBG_LOG
    // LOG_ATTACH_SERIAL(Serial);

    LOG_INFO(F("Not configured. STarting Config"));
#endif
    wm.startConfigPortal();
  }

  else
  {
    setupHA();

    if (!wm.autoConnect())
    {

#ifdef DBG_LOG
      // LOG_ATTACH_SERIAL(Serial);

      LOG_ERROR(F("Failed to connect to Wi-Fi and hit timeout."));

#endif
    }
    else
    {
      // WiFi.begin();
      wm.startWebPortal(); // Keeps portal accessible on the Wi-Fi I
#ifdef DBG_LOG
      Serial << WiFi.localIP() << endl;
#endif
      if (haHelper.getConfig()->isDeviceConfigured())
      {
#ifdef DBG_LOG
        // LOG_ATTACH_SERIAL(Serial);

        LOG_INFO(F("Device Configured...Setting sensors"));
#endif
        setupDeviceSensor();
      }

#ifdef DBG_LOG
      // LOG_ATTACH_SERIAL(Serial);

      LOG_INFO(F("Connected"));
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
      HS_001.enableInternalPulldown();
      HS_001.setPollingInterval(250); // ms
      HS_001.setOnEdgeEvent(&onPushButton);
      disableLED();

      for (int i = SENSOR8_WARMUP_DELAY; i > 0; i--)
      {
        displaySplash(&display, i);
        delay(1000);
      }
    }

    if (digitalRead(SETUP_PIN) == LOW)
    {
      // Button pressed

#ifdef DBG_LOG

      LOG_INFO(F("Entering Setup-"));

#endif

      wm.startConfigPortal();
    }
    else
    {

#ifdef DBG_LOG

      LOG_INFO(F("Starting with Config"));

#endif

#ifdef DBG_LOG

      String macAddress = WiFi.macAddress();
      LOG_INFO(String("Wifi Host Name:") + String(WiFi.getHostname()));
      LOG_INFO(String("macAddress:") + macAddress);

#endif
    }
  }
}

void unregisterDevice()
{
  haHelper.removeDiscovery(HAPM0_3Sensor);
  haHelper.removeDiscovery(HAPM0_5Sensor);
  haHelper.removeDiscovery(HAPM1_0Sensor);
  haHelper.removeDiscovery(HAPM2_5Sensor);
  haHelper.removeDiscovery(HAPM5_0Sensor);
  haHelper.removeDiscovery(HAPM10_0Sensor);

  haHelper.removeDiscovery(HA_SP_UG_1_0Sensor);
  haHelper.removeDiscovery(HA_SP_UG_2_5Sensor);
  haHelper.removeDiscovery(HA_SP_UG_10_0Sensor);

  haHelper.removeDiscovery(HA_AE_UG_1_0Sensor);
  haHelper.removeDiscovery(HA_AE_UG_2_5Sensor);
  haHelper.removeDiscovery(HA_AE_UG_10_0Sensor);

  haHelper.removeDiscovery(HACO2Sensor);
  haHelper.removeDiscovery(HAHumiditySensor);

  haHelper.removeDiscovery(HAAQI_Sensor);
  haHelper.removeDiscovery(HACategory_Sensor);
  haHelper.removeDiscovery(HATemperatureSensor);
}

// {
//   "device":
//     {
//       "name": "hadevicez",
//       "host": "hostname",
//       "token": "abc",
//       "mqttIP": "192.168.1.23",
//       "enable":0
//     }
// }

void onMqttMessage(const char *topic, const uint8_t *payload, uint16_t length)
{
  // This callback is called when message from MQTT broker is received.
  // Please note that you should always verify if the message's topic is the one you expect.
  // For example: if (memcmp(topic, "myCustomTopic") == 0) { ... }
#ifdef DBG_LOG
  LOG_TRACE(F("New message on topic:"));

  LOG_TRACE(topic);

  LOG_TRACE(F("Data:"));

  for (int i = 0; i < length; i++)
  {
    Serial << (char)payload[i];
  }
  Serial << endl;
#endif

  DeserializationError error = deserializeJson(jsonDoc, payload);

  // Test if parsing succeeds
  if (error)
  {
    LOG_ERROR(F("deserializeJson() failed: "));
    LOG_ERROR(error.f_str());
    return;
  }

  bool mode = jsonDoc["device"]["enable"];

  if (!mode)
  {
#ifdef DBG_LOG
    LOG_TRACE(F("Removing Device from Auto Discovery."));
#endif
    unregisterDevice();
    haHelper.getConfig()->setCustom(mode);
    haHelper.getConfig()->persistConfig();
  }
  else
  {
#ifdef DBG_LOG
    LOG_INFO(F("Provisioning Device."));
#endif
    haHelper.getConfig()->setCustom(mode);
    haHelper.getConfig()->persistConfig();
    mqtt.disconnect();
    mqtt.begin(haHelper.getMqttIP());
  }
}

void onMqttConnected()
{

  String tmp = haHelper.getSubscribePreTopic();
  // You can subscribe to custom topic if you need

  mqtt.subscribe(tmp.c_str());
  disableLED();
#ifdef DBG_LOG
  LOG_TRACE(F("Connected to the broker!"));

  LOG_TRACE((String(F("Suscribed to ")) + tmp).c_str());
#endif
}

void onMqttDisconnected()
{
#ifdef DBG_LOG
  LOG_TRACE(F("Disconnected from the broker!"));
#endif

  faultLEDTmr.setDelay(MQTT_FAULT_LED_RATE);
  faultLEDTmr.resume();
}

void onMqttStateChanged(HAMqtt::ConnectionState state)
{

#ifdef DBG_LOG
  LOG_TRACE(F("MQTT state changed to: "));
  LOG_TRACE(static_cast<int8_t>(state));
#endif
}

void loop()
{
  readT_RH_CO2Tmr.refresh();
  faultLEDTmr.refresh();
  pm25Sensor.refresh();
  refreshDisplayTmr.refresh();
  HS_001.refresh();

  wm.process();

  idleTmr.refresh();
  mqtt.loop();
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
  // char bannerBuff[20];
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);

  display->drawString(0, 0, aqiUS[airQualityData.category]);
  display->drawString(0, 38, displayID);
}

void displayHome(SSD1306Wire *display)
{

  char bannerBuff[20];
  display->clear();
  displayOverlay(display);
  display->setFont(ArialMT_Plain_10);
  display->setTextAlignment(TEXT_ALIGN_LEFT);

  sprintf(bannerBuff, "CO2: %d", co2Level);
  display->drawString(0, 9, bannerBuff);

  sprintf(bannerBuff, "T: %.1f", ambientTemperature);
  display->drawString(15, 18, bannerBuff);

  sprintf(bannerBuff, "H: %.1f", ambientRelativeHumidity);
  display->drawString(15, 27, bannerBuff);
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
  display->drawString(0, 9, bannerBuff);

  sprintf(bannerBuff, "PM2.5:%d", pm25Data.PM_SP_UG_2_5);
  display->drawString(0, 18, bannerBuff);

  sprintf(bannerBuff, "PM10:%d", pm25Data.PM_SP_UG_10_0);
  display->drawString(4, 27, bannerBuff);
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
  display->drawString(0, 9, bannerBuff);

  sprintf(bannerBuff, ">0.5um: %d", pm25Data.PM_TOTALPARTICLES_0_5);
  display->drawString(0, 18, bannerBuff);

  sprintf(bannerBuff, ">1.0um: %d", pm25Data.PM_TOTALPARTICLES_1_0);
  display->drawString(0, 27, bannerBuff);
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
  display->drawString(0, 9, bannerBuff);

  sprintf(bannerBuff, ">5.0um: %d", pm25Data.PM_TOTALPARTICLES_5_0);
  display->drawString(0, 18, bannerBuff);

  sprintf(bannerBuff, ">10.um: %d", pm25Data.PM_TOTALPARTICLES_10_0);
  display->drawString(0, 25, bannerBuff);
  display->display();
}

void onPushButton(bool state, int aPin)
{
#ifdef DBG_LOG

  HS_001.serialize(&Serial, true);
  // Serial << "isDisplayEnabled:" << isDisplayEnabled <<endl;

#endif
  // if( HS_001.getSample() == LOW)
  isDisplayEnabled = !isDisplayEnabled;
  onOLEDRefresh();
}

void onPM25ReadReady()
{

    airQualityData.category = UNKNOWN;
    airQualityData.index = -1;

    if (!pm25Sensor.readUntil(pm25Data))
    {
  #ifdef DBG_LOG
      Serial << "Could not read from PM25 sensor" << endl;
  #endif
    }
    else
    {
      airQualityData = aqi.get_AQI(pm25Data.PM_SP_UG_1_0, pm25Data.PM_SP_UG_2_5);
    }
}

void onreadT_RH_CO2Tmr()
{
  co2Level = co2Sensor.getCO2(3);
  ambientTemperature = sht31.readTemperature();
  ambientRelativeHumidity = sht31.readHumidity();

#if defined(DBG_LOG) && defined(DBG_SENSOR)

  Serial << "CO2 Sensair:" << co2Level << " ppm" << endl;
  Serial << "Temperature:" << ambientTemperature << "C";
  Serial << "Relative Humidity:" << ambientRelativeHumidity << "%" << endl;

#endif
}

void onfaultLEDTmrBlink()
{
#ifdef DBG_LOG
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
  if (isDisplayEnabled)
  {
    frames[currentFrame - 1](&display);
    currentFrame++;
    if (currentFrame > frameCount)
      currentFrame = 1;
  }
  else
  {
    display.clear();
    display.display();
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

  Serial << "AQI Index:" << airQualityData.index << " Category:" << airQualityData.category << " name:" << aqiUS[airQualityData.category] << endl;
}
