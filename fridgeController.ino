#include <ArduinoJson.h>
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

// fridge globals
// OneWire bus Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2
#define LIGHT_RELAY_PIN 38
#define COOLER_RELAY_PIN 39
#define CONFIG_VERSION "FC1"
#define CONFIG_START 32
#define PIEZO_PIN 40

typedef struct
{
  String deviceName;
  uint8_t deviceAddress[8];
  float tempC;
  String error;
} tempSensor;

typedef struct
{
  char version[4];
  int8_t desiredTemp;
  bool lightIsOn;
  bool audioIsOn;
} settings;

tempSensor tempSensors[2] =
    {
        {"sensor 1", {0x28, 0x59, 0x60, 0x79, 0x97, 0x5, 0x3, 0xC3}, 0.0},
        {"sensor 2", {0x28, 0xA0, 0x3E, 0x79, 0x97, 0x5, 0x3, 0x10}, 0.0}};

settings Settings = {CONFIG_VERSION, 15, false};

float averageCurrentTemp = 17;
String globalError;
bool coolerIsOn = false;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// serial communication
const uint16_t MAX_JSON_SIZE = 500;
char receivedChars[MAX_JSON_SIZE]; // an array to store the received data

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long update_period = 7000;

enum ACTION_TYPE
{
  GET_UPDATE,
  TEMP_UP,
  TEMP_DOWN,
  LIGHTS_ON,
  LIGHTS_OFF,
  AUDIO_ON,
  AUDIO_OFF
};

void sendStatusViaSerial()
{

  StaticJsonBuffer<MAX_JSON_SIZE> jsonBuffer;
  StaticJsonBuffer<MAX_JSON_SIZE> tempBuffer;

  JsonObject &root = jsonBuffer.createObject();
  root["DesiredTemp"] = Settings.desiredTemp;
  root["AverageCurrentTemp"] = averageCurrentTemp;
  root["LightIsOn"] = Settings.lightIsOn;
  root["CoolerIsOn"] = coolerIsOn;
  root["AudioIsOn"] = Settings.audioIsOn;
  if (globalError != "") {
    root["GlobalError"] = globalError;
  }
  JsonArray &sensorsJsonArray = root.createNestedArray("Sensors");
  for (uint8_t i = 0; i < sizeof(tempSensors) / sizeof(tempSensor); i++)
  {
    JsonObject &sensorData = tempBuffer.createObject();
    sensorData["Name"] = tempSensors[i].deviceName;
    sensorData["TempC"] = tempSensors[i].tempC;
    if (tempSensors[i].error != "")
    {
      sensorData["Error"] = tempSensors[i].error;
    }
    sensorsJsonArray.add(sensorData);
  }

  root.prettyPrintTo(Serial);
}

//  {"ACTION":0}

void refreshTemperatures()
{
  sensors.requestTemperatures();
  for (uint8_t i = 0; i < sizeof(tempSensors) / sizeof(tempSensor); i++)
  {
    float tempC = sensors.getTempC(tempSensors[i].deviceAddress);
    if (tempC == DEVICE_DISCONNECTED_C)
    {
      tempSensors[i].error = "Could not read temperature sensor data";
      errorBeep();
    }
    else
    {
      tempSensors[i].error = "";
    }
    tempSensors[i].tempC = tempC;
  }
  calcAverageTemp();

  //Serial.println("refreshed");
}

void calcAverageTemp()
{
  float sum = 0;
  float count = 0;
  for (uint8_t i = 0; i < sizeof(tempSensors) / sizeof(tempSensor); i++)
  {
    if (tempSensors[i].tempC == DEVICE_DISCONNECTED_C)
    {
      continue;
    }
    sum += tempSensors[i].tempC;
    count++;
  }
  averageCurrentTemp = sum / count;
}

void setup()
{

  // init stuff
  Serial.begin(9600);
  //Serial.println("Starting fridgeController.ino...");
  loadConfig();

  // relays
  pinMode(LIGHT_RELAY_PIN, OUTPUT);
  pinMode(COOLER_RELAY_PIN, OUTPUT);
  writeToLights();

  // sensors
  refreshTemperatures();

  // timer
  startMillis = millis();
}
void loop()
{
  receiveSerial();
  writeToLights();
  refreshTemperatures();
  setCoolerMode();
  writeToCooler();
}

void setCoolerMode()
{
  currentMillis = millis();
  if (currentMillis - startMillis >= update_period)
  {
    coolerIsOn = averageCurrentTemp > Settings.desiredTemp;
    startMillis = currentMillis;
  }
}

void writeToCooler()
{
  digitalWrite(COOLER_RELAY_PIN, coolerIsOn ? HIGH : LOW);
}

void writeToLights()
{
  digitalWrite(LIGHT_RELAY_PIN, Settings.lightIsOn ? HIGH : LOW);
}

void loadConfig()
{
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
    for (unsigned int t = 0; t < sizeof(settings); t++)
      *((char *)&Settings + t) = EEPROM.read(CONFIG_START + t);
}

void saveConfig()
{
  for (unsigned int t = 0; t < sizeof(settings); t++)
    EEPROM.write(CONFIG_START + t, *((char *)&Settings + t));
}

void beep()
{
  if (Settings.audioIsOn)
  {
    tone(PIEZO_PIN, 1000, 120);
  }
}

void errorBeep()
{
  if (Settings.audioIsOn)
  {
    tone(PIEZO_PIN, 3000, 50);
  }
}

boolean newData = false;
void receiveSerial()
{
  static uint16_t ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (rc != endMarker)
    {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= MAX_JSON_SIZE)
      {
        ndx = MAX_JSON_SIZE - 1;
      }
    }
    else
    {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
      // save strings
      processJSON();
      sendStatusViaSerial();
    }
  }
}

void processJSON()
{

  //Serial.print("Processing JSON ... ");
  StaticJsonBuffer<MAX_JSON_SIZE> jsonBuffer;
  JsonObject &root = jsonBuffer.parseObject(receivedChars);
  if (!root.success())
  {
    globalError = "JSON parseObject() FAILED: Received: ";
    globalError += receivedChars;
    newData = false;
    return;
  }

  //Serial.println("Done!");
  globalError = "";

  // do something with actions
  char action = root["ACTION"];
  switch (action)
  {
  case GET_UPDATE:
    //Serial.println("get update");
    refreshTemperatures();
    break;
  case TEMP_UP:
    //Serial.println("temp up");
    Settings.desiredTemp++;
    beep();
    saveConfig();
    break;
  case TEMP_DOWN:
    //Serial.println("temp down");
    Settings.desiredTemp--;
    beep();
    saveConfig();
    break;
  case LIGHTS_ON:
    //Serial.println("light on");
    Settings.lightIsOn = true;
    beep();
    saveConfig();
    break;
  case LIGHTS_OFF:
    //Serial.println("light off");
    Settings.lightIsOn = false;
    beep();
    saveConfig();
    break;
  case AUDIO_ON:
  Settings.audioIsOn = true;  
    beep();
    saveConfig();
    break;
  case AUDIO_OFF:
    Settings.audioIsOn = false;  
    beep();
    saveConfig();
    break;  
  default:
    //Serial.println("Invalid Option");
    errorBeep();
    break;
  }
  newData = false;
}

// {"ACTION":0}