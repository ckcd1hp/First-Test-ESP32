#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ESP32Time.h>
#include <WebSerial.h>
#include <AsyncElegantOTA.h>

#include "config.h"
#include "pins.h"
#include "waterLevel.h"
#include "dhtReadings.h"

#define UTC_OFFSET_IN_SECONDS -36000 // offset from greenwich time (Hawaii is UTC-10)
#define NTP_SYNC_HOUR 4
#define NTP_SYNC_MINUTE 0
#define NTP_SYNC_SECOND 0
#define WIFI_RETRY_WAIT_TIME 300000       // 5 minutes in milliseconds
#define NTP_UPDATE_INTERVAL 1800000       // 30 min in milliseconds (minimum retry time, normally daily)
#define ADC_SAMPLING_INTERVAL 50          // 50 milliseconds is 20 samples in 1 second
#define STATUS_UPDATE_INTERVAL 10000      // 10 seconds
#define WATER_LEVEL_UPDATE_INTERVAL 60000 // 1 minute
#define DHT_UPDATE_INTERVAL 900000        // 15 minutes
#define SOUND_SPEED 0.0343                // cm/microsecond
#define NUM_SAMPLES 20                    // number of samples to take for current sensor
#define CURRENT_GAIN 3                    // multiply current by 3

// #define DEBUG

#ifdef DEBUG
#define DebugLog(message)      \
  WebSerial.print("[DEBUG] "); \
  WebSerial.println(message)
#else
#define DebugLog(message)
#endif

// function declarations
void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info);                                  // on connect to Wifi
void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info);                                             // on IP received from Wifi
void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info);                               // on disconnect from Wifi
void updateAndSyncTime();                                                                            // update time from NTP server and sync to RTC
String processor(const String &var);                                                                 // update web page with variables
unsigned long setInterval(void (*callback)(), unsigned long previousMillis, unsigned long interval); // run function at interval

void overridePump(int pump_pin, int state, int time); // put a pump in override
void setPumpAuto(int pump_pin);                       //  set a pump back to auto
void controlPumps(int currentHour, int currentMin);   // control water pumps in auto or override
void checkPumpAlarms();                               // check if pump status doesn't match command
void updatePumpStatuses();                            // update web with pump statuses
void checkOverrideStatuses(int currentSecond);        // check override statuses every second
void sampleCurrent();                                 // sample current every 50 milliseconds

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_IN_SECONDS, NTP_UPDATE_INTERVAL);
ESP32Time rtc; // no offset, as that is already added from NTPClient
bool rtcUpdated = false;
String lastNTPSync = "";

// time counters
unsigned long dhtMillisCounter = 0;
unsigned long wifiPrevMillis = 0;
unsigned long adcSamplingMillisCounter = 0;
unsigned long statusUpdateMillisCounter = 0;
unsigned long waterLevelMillisCounter = 0;
// sampling variables
bool beginSampling = false;
int samplingCounter = 0;
float pump1Samples = 0.0;
float pump2Samples = 0.0;
float airPumpSamples = 0.0;
// time variables
unsigned long now;
int previousMinute = -1; // track the last minute to run functions on the new minute
int previousSecond = -1; // track the last second to run functions on the new second

// create AsyncWebServer on port 80
AsyncWebServer server(80);
// Create an Event Source on /events
AsyncEventSource events("/events");

String ledState;
bool pump1Command = false;
bool pump1Override = false;
bool pump1Status = false;
unsigned long pump1OverrideTimeEpochEnd = 0; // if pump overriden for 5 min, this will be set to current epoch + 5*60
bool pump2Command = false;
bool pump2Override = false;
bool pump2Status = false;
unsigned long pump2OverrideTimeEpochEnd = 0; // if pump overriden for 5 min, this will be set to current epoch + 5*60
bool airPumpCommand = false;                 // toggle
bool airPumpOverride = false;
bool airPumpStatus = false;
unsigned long airPumpOverrideTimeEpochEnd = 0; // if pump overriden for 5 min, this will be set to current epoch + 5*60
float mvPerAmp = 0.185;                        // sensitivity for ACS712 5A current sensor
bool readyToConnectWifi = true;                // ready to try connecting to wifi
// GET REQUEST PARAMETERS
const char *PARAM_OUTPUT = "output";
const char *PARAM_STATE = "state";
const char *PARAM_TIME = "time";

// only update status once a min on web server when there is an override timer
bool pump1StatusUpdated = false;
bool pump2StatusUpdated = false;
bool airPumpStatusUpdated = false;

// ALARMS
bool pump1Alarm = false;
bool pump2Alarm = false;
bool airPumpAlarm = false;
// alarm after 1 minute of command/status mismatch
unsigned long pump1AlarmTimeEpochEnd = 0;
unsigned long pump2AlarmTimeEpochEnd = 0;
unsigned long airPumpAlarmTimeEpochEnd = 0;
// temps
extern float f, hif, h;

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup begin");
  // set pinout
  pinMode(LED_PIN, OUTPUT);
  pinMode(WATER_PUMP_1_PIN, OUTPUT);
  pinMode(WATER_PUMP_2_PIN, OUTPUT);
  pinMode(AIR_PUMP_PIN, OUTPUT);
  // water pump current pins are input only (34 and 35) and don't need to be set
  pinMode(AIR_PUMP_CURRENT, INPUT);
  dht.begin();
  ultrasonicSensor.begin();
  // ultrasonicSensor.start();

  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // delete old config
  WiFi.disconnect(true);
  delay(1000);
  // add wifi events
  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.mode(WIFI_STA); // station mode: ESP32 connects to access point
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting to WIFI");
  delay(10000);
  timeClient.begin();

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", String(), false, processor); });

  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/style.css", "text/css"); });
  // Route to load script.js file
  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/script.js", "text/javascript"); });

  // Route to set GPIO to HIGH
  server.on("/led2on", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    digitalWrite(LED_PIN, HIGH);    
    request->send(SPIFFS, "/index.html", String(), false, processor); });

  // Route to set GPIO to LOW
  server.on("/led2off", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    digitalWrite(LED_PIN, LOW);    
    request->send(SPIFFS, "/index.html", String(), false, processor); });

  server.on("/override", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    int output, state, time;
    //String debug = "";
    // GET input1 value on <ESP_IP>/override?output=<output>&state=<state&time=<time>
    if (request->hasParam(PARAM_OUTPUT) && request->hasParam(PARAM_STATE) && request->hasParam(PARAM_TIME))
    {
      output = request->getParam(PARAM_OUTPUT)->value().toInt();
      state = request->getParam(PARAM_STATE)->value().toInt();
      time = request->getParam(PARAM_TIME)->value().toInt();
      overridePump(output, state, time);
      //debug = "Set pin " + String(output) + " to " + (state == 1) ? "On " : "Off " + (time > 60) ? "permanently" : "for " + String(time) + " min"; 
    }
    else {
      //debug = "No message sent";
    }
    //Serial.println(debug);
    request->send(200, "text/plain", "OK"); });

  server.on("/auto", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    int output;
    //String debug = "";
    // GET input1 value on <ESP_IP>/auto?output=<output>
    if (request->hasParam(PARAM_OUTPUT))
    {
      output = request->getParam(PARAM_OUTPUT)->value().toInt();
      setPumpAuto(output);
    //  debug = "Set pin " + String(output) + " to auto"; 
    }
    else {
     // debug = "No message sent";
    }
    //Serial.println(debug);
    request->send(200, "text/plain", "OK"); });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client)
                   {
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000); });
  server.addHandler(&events);
  AsyncElegantOTA.begin(&server);
  WebSerial.begin(&server);
  server.begin();

  // initial dht reading
  getDhtReadings();
}

void loop()
{
  // check if ultrasonic sensor has distance reading
  if (ultrasonicSensor.isFinished())
  {
    displayWaterLevel(ultrasonicSensor.getRange());
  }
  now = millis();

  // check if it is time to sample current
  if (now - statusUpdateMillisCounter >= STATUS_UPDATE_INTERVAL)
  {
    beginSampling = true;
    statusUpdateMillisCounter += STATUS_UPDATE_INTERVAL;
  }
  if (beginSampling)
  {
    // sample current sensors every 50ms
    if (now - adcSamplingMillisCounter >= ADC_SAMPLING_INTERVAL)
    {
      sampleCurrent();
      adcSamplingMillisCounter += ADC_SAMPLING_INTERVAL;
    }
  }

  // get dht readings every set interval (default 15 min)
  dhtMillisCounter = setInterval(getDhtReadings, dhtMillisCounter, DHT_UPDATE_INTERVAL);
  // get water level every set interval (default 15 min)
  waterLevelMillisCounter = setInterval(checkWaterLevel, waterLevelMillisCounter, WATER_LEVEL_UPDATE_INTERVAL);

  // check counter if connecting to wifi
  if (!readyToConnectWifi)
  {
    if (now - wifiPrevMillis > WIFI_RETRY_WAIT_TIME)
    {
      readyToConnectWifi = true;
      wifiPrevMillis += WIFI_RETRY_WAIT_TIME;
    }
  }
  else if (readyToConnectWifi and WiFi.status() != WL_CONNECTED)
  {
    // ready to connect
    delay(5000); // will attempt to reconnect before disconnect event even fires
    Serial.println("Reconnecting to WiFi");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    wifiPrevMillis = now; // reset timer
    readyToConnectWifi = false;
  }

  int currentHour = rtc.getHour(true);
  int currentMin = rtc.getMinute();
  int currentSec = rtc.getSecond();

  // check if minute has changed
  if (currentMin != previousMinute)
  {
    // control pump in auto
    controlPumps(currentHour, currentMin);
    previousMinute = currentMin;
  }
  if (currentSec != previousSecond)
  {
    // check override statuses once a second
    checkOverrideStatuses(currentSec);
    // check pump alarms
    checkPumpAlarms();
    previousSecond = currentSec;
  }
  // Update time using NTP at same time everyday (getHour(true) outputs 0-23)
  if (currentHour == NTP_SYNC_HOUR and currentMin == NTP_SYNC_MINUTE and currentSec == NTP_SYNC_SECOND)
  {
    if (!rtcUpdated)
    {
      updateAndSyncTime();
    }
  }
  else
  {
    rtcUpdated = false;
  }
}

void updateAndSyncTime()
{
  if (timeClient.update())
  {
    // successful update
    Serial.println("Recieved updated time from NTP!");
    // set RTC time
    rtc.setTime(timeClient.getEpochTime());
    lastNTPSync = rtc.getTime("%A, %B %d %Y %I:%M %p");
    // Serial.println("RTC: " + lastNTPSync);
    rtcUpdated = true;
  }
  else
  {
    // unsuccessful update, display current unsynced RTC time
    Serial.println("Unable to connect to NTP or already updated within the last 30 minutes");
    Serial.println("RTC: " + rtc.getTime("%A, %B %d %Y %I:%M %p"));
  }
}
String processor(const String &var)
{
  if (var == "GPIO_STATE")
  {
    if (digitalRead(LED_PIN))
    {
      ledState = "ON";
    }
    else
    {
      ledState = "OFF";
    }
    return ledState;
  }
  else if (var == "CURRENT_TIME")
  {
    return rtc.getTime("%A, %B %d %Y %I:%M %p");
  }
  else if (var == "LAST_SYNC_TIME")
  {
    return lastNTPSync;
  }
  else if (var == "TEMPERATURE")
  {
    // get current dht readings to update webpage
    // only needs to run once and temperature is read first
    getDhtReadings();
    return String(f, 1);
  }
  else if (var == "HUMIDITY")
  {
    return String(h, 0);
  }
  else if (var == "HEAT_INDEX")
  {
    return String(hif, 1);
  }
  else if (var == "WATER_LEVEL")
  {
    return ("Checking...");
  }
  else if (var == "PUMP_1_COMMAND")
  {
    String command = "";
    if (digitalRead(WATER_PUMP_1_PIN))
    {
      command = "On ";
    }
    else
    {
      command = "Off ";
    }
    if (pump1Override)
    {
      String timeLeft = "Permanent)";
      if (pump1OverrideTimeEpochEnd > 0)
      {
        timeLeft = String((pump1OverrideTimeEpochEnd - rtc.getEpoch()) / 60) + " min)"; // time left in minutes
      }
      return command + "(Override " + timeLeft;
    }
    return command + "(Auto)";
  }
  else if (var == "PUMP_2_COMMAND")
  {
    String command = "";
    if (digitalRead(WATER_PUMP_2_PIN))
    {
      command = "On ";
    }
    else
    {
      command = "Off ";
    }
    if (pump2Override)
    {
      String timeLeft = "Permanent)";
      if (pump2OverrideTimeEpochEnd > 0)
      {
        timeLeft = String((pump2OverrideTimeEpochEnd - rtc.getEpoch()) / 60) + " min)"; // time left in minutes
      }
      return command + "(Override " + timeLeft;
    }
    return command + "(Auto)";
  }
  else if (var == "AIR_PUMP_COMMAND")
  {
    String command = "";
    if (digitalRead(AIR_PUMP_PIN))
    {
      command = "On ";
    }
    else
    {
      command = "Off ";
    }
    if (airPumpOverride)
    {
      String timeLeft = "Permanent)";
      if (airPumpOverrideTimeEpochEnd > 0)
      {
        timeLeft = String((airPumpOverrideTimeEpochEnd - rtc.getEpoch()) / 60) + " min)"; // time left in minutes
      }
      return command + "(Override " + timeLeft;
    }
    return command + "(Auto)";
  }
  return String();
}
void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // mdns responder for esp32.local
  if (MDNS.begin("esp32"))
  {
    Serial.println("MDNS responder started, accessible via esp32.local");
  }
  delay(2000);
  // The function timeClient.update() syncs the local time to the NTP server. In the video I call this in the main loop. However, NTP servers dont like it if
  // they get pinged all the time, so I recommend to only re-sync to the NTP server occasionally. In this example code we only call this function once in the
  // setup() and you will see that in the loop the local time is automatically updated. Of course the ESP/Arduino does not have an infinitely accurate clock,
  // so if the exact time is very important you will need to re-sync once in a while.
  updateAndSyncTime(); // anytime esp32 reconnects to wifi it will attempt to sync time
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  WiFi.disconnect(true);
}

// for calling a function every interval
unsigned long setInterval(void (*callback)(), unsigned long previousMillis, unsigned long interval)
{
  if (millis() - previousMillis >= interval)
  {
    callback();
    previousMillis += interval;
  }
  return previousMillis;
}
void overridePump(int pump_pin, int state, int time)
{
  if (pump_pin == WATER_PUMP_1_PIN)
  {
    pump1Override = true;
    pump1Command = state ? true : false;
    if (time > 60)
    {
      // permanent override
      pump1OverrideTimeEpochEnd = 0;
    }
    else
    {
      pump1OverrideTimeEpochEnd = rtc.getEpoch() + (time * 60); // time in minutes to seconds
    }
  }
  else if (pump_pin == WATER_PUMP_2_PIN)
  {
    pump2Override = true;
    pump2Command = state ? true : false;
    if (time > 60)
    {
      // permanent override
      pump2OverrideTimeEpochEnd = 0;
    }
    else
    {
      pump2OverrideTimeEpochEnd = rtc.getEpoch() + (time * 60); // time in minutes to seconds
    }
  }
  else
  {
    airPumpOverride = true;
    airPumpCommand = state ? true : false;
    if (time > 60)
    {
      // permanent override
      airPumpOverrideTimeEpochEnd = 0;
    }
    else
    {
      airPumpOverrideTimeEpochEnd = rtc.getEpoch() + (time * 60); // time in minutes to seconds
    }
  }
  digitalWrite(pump_pin, state);
}
void setPumpAuto(int pump_pin)
{
  int currentHour = rtc.getHour(true); // current time
  if (pump_pin == WATER_PUMP_1_PIN)
  {
    if (currentHour >= 6 and currentHour < 12)
    {
      pump1Command = true;
      digitalWrite(WATER_PUMP_1_PIN, HIGH);
    }
    else
    {
      pump1Command = false;
      digitalWrite(WATER_PUMP_1_PIN, LOW);
    }

    pump1Override = false;
    pump1OverrideTimeEpochEnd = 0;
    // Send Events to the Web Client with the Sensor Readings
    String command = (pump1Command) ? " On " : " Off ";
    String pumpCommand = command + "(Auto)";
    events.send(pumpCommand.c_str(), "pump1Command", millis());
  }
  else if (pump_pin == WATER_PUMP_2_PIN)
  {
    if (currentHour >= 12 and currentHour < 18)
    {
      pump2Command = true;
      digitalWrite(WATER_PUMP_2_PIN, HIGH);
    }
    else
    {
      pump2Command = false;
      digitalWrite(WATER_PUMP_2_PIN, LOW);
    }
    pump2Override = false;
    pump2OverrideTimeEpochEnd = 0;
    // Send Events to the Web Client with the Sensor Readings
    String command = (pump2Command) ? " On " : " Off ";
    String pumpCommand = command + "(Auto)";
    events.send(pumpCommand.c_str(), "pump2Command", millis());
  }
  else
  {
    airPumpCommand = true;
    digitalWrite(AIR_PUMP_PIN, HIGH); // switching back to auto will just turn it on, it'll go back to 15 min on/off
    airPumpOverride = false;
    airPumpOverrideTimeEpochEnd = 0;
    // Send Events to the Web Client with the Sensor Readings
    String pumpCommand = "On (Auto)";
    events.send(pumpCommand.c_str(), "airPumpCommand", millis());
  }
}
void controlPumps(int currentHour, int currentMin)
{
  // function runs once a minute
  // water pump 1
  if (!pump1Override)
  {
    // Run water pump 1 from 6am to 12pm continuously.  The other 12 hours, the pump will run for 1 min on the hour
    if (currentHour >= 6 and currentHour < 12)
      pump1Command = true;
    else if (currentMin == 0)
      pump1Command = true;
    else
      pump1Command = false;
  }
  // water pump 2
  if (!pump2Override)
  { // auto mode
    // Run water pump 2 from 12pm to 6pm continuously.  The other 12 hours, the pump will run for 1 min on the half hour
    if (currentHour >= 12 and currentHour < 18)
      pump2Command = true;
    else if (currentMin == 30)
      pump2Command = true;
    else
      pump2Command = false;
  }
  // air pump
  if (!airPumpOverride)
  { // auto mode
    // Toggle air pump every 15 minutes
    if (currentMin % 15 == 0)
    {
      airPumpCommand = !airPumpCommand;
      // Send Events to the Web Client with Air Pump Command
      String command = (airPumpCommand) ? " On " : " Off ";
      String pumpCommand = command + "(Auto)";
      events.send(pumpCommand.c_str(), "airPumpCommand", millis());
    }
  }
  // set pump1 command
  digitalWrite(WATER_PUMP_1_PIN, pump1Command ? HIGH : LOW);
  // set pump2 command
  digitalWrite(WATER_PUMP_2_PIN, pump2Command ? HIGH : LOW);
  // set air pump command
  digitalWrite(AIR_PUMP_PIN, airPumpCommand ? HIGH : LOW);
}
// check override statuses every second
void checkOverrideStatuses(int currentSecond)
{
  unsigned long epoch = rtc.getEpoch();
  if (pump1Override)
  {
    // pump1 is in override for set duration (set by user from webpage)
    // update web page every minute
    if (currentSecond == 0)
    {
      // if it is 0 it is in permanent override
      if (pump1OverrideTimeEpochEnd != 0)
      {
        // Send Events to the Web Client with the Sensor Readings
        String command = (pump1Command) ? "On " : "Off ";
        unsigned long timeLeftMin = (pump1OverrideTimeEpochEnd - epoch) / 60;
        String timeLeft = command + "(Override " + String(timeLeftMin) + " min)";
        if (timeLeftMin != 0) // avoid sending 0 min left
          events.send(timeLeft.c_str(), "pump1Command", millis());
      }
    }
    // if epoch has passed the end time, set pump to auto
    if (epoch >= pump1OverrideTimeEpochEnd and pump1OverrideTimeEpochEnd != 0)
    {
      setPumpAuto(WATER_PUMP_1_PIN);
    }
  }
  if (pump2Override)
  {
    // pump2 is in override for set duration (set by user from webpage)
    // update web page every minute
    if (currentSecond == 0)
    {
      if (pump2OverrideTimeEpochEnd != 0)
      {
        // Send Events to the Web Client with the Sensor Readings
        String command = (pump2Command) ? "On " : "Off ";
        unsigned long timeLeftMin = (pump2OverrideTimeEpochEnd - epoch) / 60;
        String timeLeft = command + "(Override " + String(timeLeftMin) + " min)";
        if (timeLeftMin != 0) // avoid sending 0 min left
          events.send(timeLeft.c_str(), "pump2Command", millis());
      }
    }
    // if pump2OverrideTimeEpochEnd is 0 and pump is in override, then override is permanent
    if (epoch >= pump2OverrideTimeEpochEnd and pump2OverrideTimeEpochEnd != 0)
    {
      setPumpAuto(WATER_PUMP_2_PIN);
    }
  }
  if (airPumpOverride)
  {
    // air pump is in override for set duration (set by user from webpage)
    // update web page every minute
    if (currentSecond == 0)
    {
      if (airPumpOverrideTimeEpochEnd != 0)
      {
        // Send Events to the Web Client with the Sensor Readings
        String command = (airPumpCommand) ? "On " : "Off ";
        unsigned long timeLeftMin = (airPumpOverrideTimeEpochEnd - epoch) / 60;
        String timeLeft = command + "(Override " + String(timeLeftMin) + " min)";
        if (timeLeftMin != 0) // avoid sending 0 min left
          events.send(timeLeft.c_str(), "airPumpCommand", millis());
      }
    }
    // if pump2OverrideTimeEpochEnd is 0 and pump is in override, then override is permanent
    if (epoch >= airPumpOverrideTimeEpochEnd and airPumpOverrideTimeEpochEnd != 0)
    {
      setPumpAuto(AIR_PUMP_PIN);
    }
  }
}
void updatePumpStatuses()
{
  // Send Events to the Web Client with the pump statuses (every 10 seconds)
  String p1String = (pump1Status) ? "<span class = \"status online\"></ span>" : "<span class=\" status offline \"></span> ";
  String p2String = (pump2Status) ? "<span class = \"status online\"></ span>" : "<span class=\" status offline \"></span> ";
  String airPString = (airPumpStatus) ? "<span class = \"status online\"></ span>" : "<span class=\" status offline \"></span> ";
  events.send(p1String.c_str(), "pump1Status", millis());
  events.send(p2String.c_str(), "pump2Status", millis());
  events.send(airPString.c_str(), "airPumpStatus", millis());
}
void sampleCurrent()
{
  // sample current every 50 milliseconds
  if (samplingCounter < NUM_SAMPLES)
  {
    // add to samples
    pump1Samples += analogRead(WATER_PUMP_1_CURRENT);
    pump2Samples += analogRead(WATER_PUMP_2_CURRENT);
    airPumpSamples += analogRead(AIR_PUMP_CURRENT);
    samplingCounter++;
  }
  else
  {
    // calculate average
    pump1Samples /= NUM_SAMPLES;
    pump2Samples /= NUM_SAMPLES;
    airPumpSamples /= NUM_SAMPLES;
    // convert to voltage
    pump1Samples = pump1Samples * (3.3 / 4095) - 1.52;
    pump2Samples = pump2Samples * (3.3 / 4095) - 1.52;
    airPumpSamples = airPumpSamples * (3.3 / 4095) - 1.52;
    // calculate amps
    float pump1Current = pump1Samples * CURRENT_GAIN / mvPerAmp;
    float pump2Current = pump2Samples * CURRENT_GAIN / mvPerAmp;
    float airPumpCurrent = airPumpSamples * CURRENT_GAIN / mvPerAmp;
    // update pump statuses
    pump1Status = (pump1Current > 0.5) ? true : false;
    pump2Status = (pump2Current > 0.5) ? true : false;
    airPumpStatus = (airPumpCurrent > 0.3) ? true : false;
    // write status to web
    updatePumpStatuses();
    // debug
    DebugLog("Pump 1 Current: " + String(pump1Current));
    DebugLog("Pump 2 Current: " + String(pump2Current));
    DebugLog("Air Pump Current: " + String(airPumpCurrent));
    // reset counter
    samplingCounter = 0;
    // reset samples
    pump1Samples = 0.0;
    pump2Samples = 0.0;
    airPumpSamples = 0.0;
    // reset begin sampling
    beginSampling = false;
  }
}
void checkPumpAlarms()
{
  if (pump1Command != pump1Status)
  {
    if (pump1AlarmTimeEpochEnd == 0)
    {
      // first instance of mismatch, start timer if not already in alarm
      if (!pump1Alarm)
      {
        pump1AlarmTimeEpochEnd = rtc.getEpoch() + 60; // 1 minute timer
        Serial.println("Starting pump1 alarm timer");
      }
    }
    else
    {
      if (rtc.getEpoch() >= pump1AlarmTimeEpochEnd)
      {
        // set alarm
        pump1Alarm = true;
        pump1AlarmTimeEpochEnd = 0;
        String p1Alarm = "<i class = \"fas fa-bell\" style = \"color:#c81919;\"></ i> Water Pump 1";
        events.send(p1Alarm.c_str(), "waterPump1Header", millis());
        Serial.println("Pump1 alarm active");
      }
    }
  }
  else
  {
    // reset alarm
    // as soon as status matches, clear timer
    pump1AlarmTimeEpochEnd = 0;
    if (pump1Alarm)
    {
      pump1Alarm = false;
      String p1 = "Water Pump 1";
      events.send(p1.c_str(), "waterPump1Header", millis());
      Serial.println("Pump1 alarm cleared");
    }
  }
  // check pump2
  if (pump2Command != pump2Status)
  {
    if (pump2AlarmTimeEpochEnd == 0)
    {
      // first instance of mismatch, start timer if not already in alarm
      if (!pump2Alarm)
      {
        pump2AlarmTimeEpochEnd = rtc.getEpoch() + 60; // 1 minute timer
        Serial.println("Starting pump2 alarm timer");
      }
    }
    else
    {
      if (rtc.getEpoch() >= pump2AlarmTimeEpochEnd)
      {
        // set alarm
        pump2Alarm = true;
        pump2AlarmTimeEpochEnd = 0;
        String p2Alarm = "<i class = \"fas fa-bell\" style = \"color:#c81919;\"></ i> Water Pump 2";
        events.send(p2Alarm.c_str(), "waterPump2Header", millis());
        Serial.println("Pump2 alarm active");
      }
    }
  }
  else
  {
    // reset alarm
    // as soon as status matches, clear timer
    pump2AlarmTimeEpochEnd = 0;
    if (pump2Alarm)
    {
      pump2Alarm = false;
      String p2 = "Water Pump 2";
      events.send(p2.c_str(), "waterPump2Header", millis());
      Serial.println("Pump2 alarm cleared");
    }
  }
  // check air pump
  if (airPumpCommand != airPumpStatus)
  {
    if (airPumpAlarmTimeEpochEnd == 0)
    {
      // first instance of mismatch, start timer if not already in alarm
      if (!airPumpAlarm)
      {
        airPumpAlarmTimeEpochEnd = rtc.getEpoch() + 60; // 1 minute timer
        Serial.println("Starting air pump alarm timer");
      }
    }
    else
    {
      if (rtc.getEpoch() >= airPumpAlarmTimeEpochEnd)
      {
        // set alarm
        airPumpAlarm = true;
        airPumpAlarmTimeEpochEnd = 0;
        String airPumpAlarm = "<i class = \"fas fa-bell\" style = \"color:#c81919;\"></ i> Air Pump";
        events.send(airPumpAlarm.c_str(), "airPumpHeader", millis());
        Serial.println("Air pump alarm active");
      }
    }
  }
  else
  {
    // reset alarm
    // as soon as status matches, clear timer
    airPumpAlarmTimeEpochEnd = 0;
    if (airPumpAlarm)
    {
      airPumpAlarm = false;
      String p = "Air Pump";
      events.send(p.c_str(), "airPumpHeader", millis());
      Serial.println("Air Pump alarm cleared");
    }
  }
}
