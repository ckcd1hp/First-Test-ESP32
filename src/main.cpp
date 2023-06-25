#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ESP32Time.h>
#include <DHT.h>

#include "config.h"

#define UTC_OFFSET_IN_SECONDS -36000 // offset from greenwich time (Hawaii is UTC-10)
#define NTP_SYNC_HOUR 4
#define NTP_SYNC_MINUTE 0
#define NTP_SYNC_SECOND 0
#define WIFI_RETRY_WAIT_TIME 300000 // 5 minutes in milliseconds
#define NTP_UPDATE_INTERVAL 1800000 // 30 min in milliseconds (minimum retry time, normally daily)

// pin definitons
#define LED_PIN 2
#define WATER_PUMP_1_PIN 22
#define WATER_PUMP_2_PIN 21
#define AIR_PUMP_PIN 19
#define DHT_PIN 23
#define WATER_PUMP_1_CURRENT 34
#define WATER_PUMP_2_CURRENT 35
#define AIR_PUMP_CURRENT 32

// function declarations
void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info);    //
void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info);               // on connect
void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info); // on disconnect from Wifi
void updateAndSyncTime();
String processor(const String &var);
unsigned long setInterval(void (*callback)(), unsigned long previousMillis, unsigned long interval);
void getDhtReadings();                                // get temp and humidity readings from dht sensor
void toggleAirPump();                                 // turn air pump on/off
void overridePump(int pump_pin, int state, int time); // put a pump in override
void setPumpAuto(int pump_pin);                       //  set a pump back to auto
void controlPumps(int currentHour, int currentMin);   // control water pumps in auto or override

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_IN_SECONDS, NTP_UPDATE_INTERVAL);
ESP32Time rtc; // no offset, as that is already added from NTPClient
bool rtcUpdated = false;

// time interval setup
int dhtInterval = 900000;
int airPumpInterval = 900000;
int adcSamplingInterval = 50; // 100 milliseconds means 10 samples in 1 second
unsigned long airPumpMillisCounter = 0;
unsigned long dhtMillisCounter = 0;
unsigned long wifiPrevMillis = 0;
unsigned long adcSamplingMillisCounter = 0;
int samplingCounter = 0;
float samples = 0.0;
unsigned long now;

// create AsyncWebServer on port 80
AsyncWebServer server(80);
// Create an Event Source on /events
AsyncEventSource events("/events");
DHT dht(DHT_PIN, DHT11);

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String ledState;
float h, f, hif;       // humidity, temp in fahrenheit, heat index fahrenheit
bool pumpLead = false; // false is pump1 lead, true is pump2 lead
bool pump1Override = false;
bool pump1Status = false;
unsigned long pump1OverrideTimeEpochEnd = 0; // if pump overriden for 5 min, this will be set to current epoch + 5*60
bool pump2Override = false;
bool pump2Status = false;
unsigned long pump2OverrideTimeEpochEnd = 0; // if pump overriden for 5 min, this will be set to current epoch + 5*60
bool airPumpCommand = false;                 // toggle
bool airPumpOverride = false;
bool airPumpStatus = false;
unsigned long airPumpOverrideTimeEpochEnd = 0; // if pump overriden for 5 min, this will be set to current epoch + 5*60
float mvPerAmp = 0.185;                        // sensitivity for ACS712 5A current sensor
bool readyToConnectWifi = true;                // ready to try connecting to wifi
const char *PARAM_OUTPUT = "output";
const char *PARAM_STATE = "state";
const char *PARAM_TIME = "time";

bool pump1StatusUpdated = false;
bool pump2StatusUpdated = false;
bool airPumpStatusUpdated = false;

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup begin");
  // set pinout
  pinMode(LED_PIN, OUTPUT);
  pinMode(WATER_PUMP_1_PIN, OUTPUT);
  dht.begin();

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
    String debug = "";
    // GET input1 value on <ESP_IP>/override?output=<output>&state=<state&time=<time>
    if (request->hasParam(PARAM_OUTPUT) && request->hasParam(PARAM_STATE) && request->hasParam(PARAM_TIME))
    {
      output = request->getParam(PARAM_OUTPUT)->value().toInt();
      state = request->getParam(PARAM_STATE)->value().toInt();
      time = request->getParam(PARAM_TIME)->value().toInt();
      overridePump(output, state, time);
      debug = "Set pin " + String(output) + " to " + (state == 1) ? "On " : "Off " + (time > 60) ? "permanently" : "for " + String(time) + " min"; 
    }
    else {
      debug = "No message sent";
    }
    Serial.println(debug);
    request->send(200, "text/plain", "OK"); });

  server.on("/auto", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    int output;
    String debug = "";
    // GET input1 value on <ESP_IP>/auto?output=<output>
    if (request->hasParam(PARAM_OUTPUT))
    {
      output = request->getParam(PARAM_OUTPUT)->value().toInt();
      setPumpAuto(output);
      debug = "Set pin " + String(output) + " to auto"; 
    }
    else {
      debug = "No message sent";
    }
    Serial.println(debug);
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
  server.begin();

  // initial dht reading
  getDhtReadings();
}

void loop()
{

  now = millis();
  if (now - adcSamplingMillisCounter >= adcSamplingInterval)
  {
    samples = samples + analogRead(WATER_PUMP_1_CURRENT);
    adcSamplingMillisCounter += adcSamplingInterval;
    samplingCounter++;
    // Serial.print("samples: ");
    // Serial.println(samples);
    // Serial.println(analogRead(WATER_PUMP_1_CURRENT) * (3.3 / 4095));
    if (samplingCounter >= 50)
    {
      float avgADC = samples / 50;
      float voltage = avgADC * 3.3 / 4095.0 - 1.55;
      float current = voltage * 2 / mvPerAmp;
      pump1Status = (current > 0.5) ? true : false;
      Serial.print("Voltage: ");
      Serial.println(voltage);
      Serial.print("Current: ");
      Serial.println(current);
      Serial.println(pump1Status);
      samplingCounter = 0;
      samples = 0.0;
    }
  }

  // get dht readings every set interval (default 15 min)
  dhtMillisCounter = setInterval(getDhtReadings, dhtMillisCounter, dhtInterval);
  // toggle air pump every set interval (default 15 min)
  airPumpMillisCounter = setInterval(toggleAirPump, airPumpMillisCounter, airPumpInterval);

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
  // controls pumps (auto vs override)
  controlPumps(currentHour, currentMin);
}

void updateAndSyncTime()
{
  if (timeClient.update())
  {
    // successful update
    Serial.println("Recieved updated time from NTP!");
    rtc.setTime(timeClient.getEpochTime());
    Serial.println("RTC: " + rtc.getTime("%A, %B %d %Y %r"));
    rtcUpdated = true;
  }
  else
  {
    Serial.println("Unable to connect to NTP or already updated within the last 30 minutes");
    Serial.println("RTC: " + rtc.getTime("%A, %B %d %Y %r"));
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
  else if (var == "TEMPERATURE")
  {
    // get current dht readings to update webpage
    // only needs to run once and temperature is read first
    getDhtReadings();
    return String(f);
  }
  else if (var == "HUMIDITY")
  {
    return String(h);
  }
  else if (var == "HEAT_INDEX")
  {
    return String(hif);
  }
  else if (var == "PUMP_1_STATUS")
  {
    String status = "Status: ";
    if (digitalRead(WATER_PUMP_1_PIN))
    {
      status += "On ";
    }
    else
    {
      status += "Off ";
    }
    if (pump1Override)
    {
      String timeLeft = "Permanent";
      if (pump1OverrideTimeEpochEnd > 0)
      {
        timeLeft = String((pump1OverrideTimeEpochEnd - rtc.getEpoch()) / 60); // time left in minutes
      }
      return status + "(Override " + timeLeft + " min)";
    }
    return status + "(Auto)";
  }
  else if (var == "PUMP_2_STATUS")
  {
    String status = "Status: ";
    if (digitalRead(WATER_PUMP_2_PIN))
    {
      status += "On ";
    }
    else
    {
      status += "Off ";
    }
    if (pump2Override)
    {
      String timeLeft = "Permanent";
      if (pump2OverrideTimeEpochEnd > 0)
      {
        timeLeft = String((pump2OverrideTimeEpochEnd - rtc.getEpoch()) / 60); // time left in minutes
      }
      return status + "(Override " + timeLeft + " min)";
    }
    return status + "(Auto)";
  }
  else if (var == "AIR_PUMP_STATUS")
  {
    String status = "Status: ";
    if (digitalRead(AIR_PUMP_PIN))
    {
      status += "On ";
    }
    else
    {
      status += "Off ";
    }
    if (airPumpOverride)
    {
      String timeLeft = "Permanent";
      if (airPumpOverrideTimeEpochEnd > 0)
      {
        timeLeft = String((airPumpOverrideTimeEpochEnd - rtc.getEpoch()) / 60); // time left in minutes
      }
      return status + "(Override " + timeLeft + " min)";
    }
    return status + "(Auto)";
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
void getDhtReadings()
{
  h = dht.readHumidity();
  f = dht.readTemperature(true); // true outputs in fahrenheit
  if (isnan(h) || isnan(f))
  {
    Serial.println("Error: Failed to read from DHT sensor!");
  }
  else
  {
    // Compute heat index in Fahrenheit
    hif = dht.computeHeatIndex(f, h);
    Serial.println((String) "Temperature: " + f + "F");
    Serial.println((String) "Humidity: " + h + "%");
    Serial.println((String) "Heat Index: " + hif + "F");
    Serial.println(rtc.getTime());
    // Send Events to the Web Client with the Sensor Readings
    events.send(String(f).c_str(), "temperature", millis());
    events.send(String(h).c_str(), "humidity", millis());
    events.send(String(hif).c_str(), "heatIndex", millis());
  }
}
void toggleAirPump()
{
  // if pump is overriden check timer
  if (airPumpOverride)
  {
    if (rtc.getEpoch() >= airPumpOverrideTimeEpochEnd and airPumpOverrideTimeEpochEnd != 0)
    {
      // timer elapsed, back to auto
      airPumpOverrideTimeEpochEnd = 0;
      airPumpOverride = false;
    }
    return;
  }
  // air pump will turn on for 15 minutes and then stay off for 15 minutes continuously
  airPumpCommand = !airPumpCommand;
  if (airPumpCommand)
  {
    digitalWrite(AIR_PUMP_PIN, HIGH);
  }
  else
  {
    digitalWrite(AIR_PUMP_PIN, LOW);
  }
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
  if (pump_pin == WATER_PUMP_1_PIN)
  {
    pump1Override = false;
    // Send Events to the Web Client with the Sensor Readings
    String status = "Status: ";
    String pumpStatus = (pump1Status) ? "On " : "Off ";
    status += pumpStatus + "(Auto)";
    events.send(status.c_str(), "pump1Status", millis());
  }
  else if (pump_pin == WATER_PUMP_2_PIN)
  {
    pump2Override = false;
    // Send Events to the Web Client with the Sensor Readings
    String status = "Status: ";
    String pumpStatus = (pump2Status) ? "On " : "Off ";
    status += pumpStatus + "(Auto)";
    events.send(status.c_str(), "pump2Status", millis());
  }
  else
  {
    airPumpOverride = false;
    // Send Events to the Web Client with the Sensor Readings
    String status = "Status: ";
    String pumpStatus = (airPumpStatus) ? "On " : "Off ";
    status += pumpStatus + "(Auto)";
    events.send(status.c_str(), "airPumpStatus", millis());
  }
}
void controlPumps(int currentHour, int currentMin)
{
  if (!pump1Override)
  { // auto mode
    // Run water pump 1 from 6am to 6pm continuously.  The other 12 hours, the pump will run for 1 min on the hour
    if (currentHour >= 6 and currentHour <= 18)
      digitalWrite(WATER_PUMP_1_PIN, HIGH);
    else if (currentMin == 0)
      digitalWrite(WATER_PUMP_1_PIN, HIGH);
    else
      digitalWrite(WATER_PUMP_1_PIN, LOW);
  }
  else
  { // pump in hand
    // pump1 is in override for set duration (set by user from webpage)

    // update web page every minute
    if (rtc.getSecond() == 0)
    {
      if (!pump1StatusUpdated)
      {
        // Send Events to the Web Client with the Sensor Readings
        String status = (pump1Status) ? "On " : "Off ";
        String timeLeft = "Status: " + status + "(Override " + String((pump1OverrideTimeEpochEnd - rtc.getEpoch()) / 60) + " min)";
        events.send(timeLeft.c_str(), "pump1Status", millis());
        pump1StatusUpdated = true;
      }
    }
    else
      pump1StatusUpdated = false;
    // if pump1OverrideTimeEpochEnd is 0 and pump is in override, then override is permanent
    if (rtc.getEpoch() >= pump1OverrideTimeEpochEnd and pump1OverrideTimeEpochEnd != 0)
    {
      // timer elapsed, back to auto
      pump1OverrideTimeEpochEnd = 0;
      pump1Override = false;
      // Send Events to the Web Client with the Sensor Readings
      String status = "Status: " + (pump1Status) ? "On " : "Off ";
      status += "(Auto)";
      events.send(status.c_str(), "pump1Status", millis());
    }
  }
  /* ----------------------------- WATER PUMP 2 -----------------------------------------*/
  if (!pump2Override)
  { // auto mode
    // Run water pump 1 from 6am to 6pm continuously.  The other 12 hours, the pump will run for 1 min on the hour
    if (currentHour >= 6 and currentHour <= 18)
      digitalWrite(WATER_PUMP_2_PIN, HIGH);
    else if (currentMin == 0)
      digitalWrite(WATER_PUMP_2_PIN, HIGH);
    else
      digitalWrite(WATER_PUMP_2_PIN, LOW);
  }
  else
  { // pump in hand
    // pump2 is in override for set duration (set by user from webpage)

    // update web page every minute
    if (rtc.getSecond() == 0)
    {
      if (!pump2StatusUpdated)
      {
        // Send Events to the Web Client with the Sensor Readings
        String status = (pump2Status) ? "On " : "Off ";
        String timeLeft = "Status: " + status + "(Override " + String((pump2OverrideTimeEpochEnd - rtc.getEpoch()) / 60) + " min)";
        events.send(timeLeft.c_str(), "pump2Status", millis());
        pump2StatusUpdated = true;
      }
    }
    else
      pump2StatusUpdated = false;
    // if pump1OverrideTimeEpochEnd is 0 and pump is in override, then override is permanent
    if (rtc.getEpoch() >= pump2OverrideTimeEpochEnd and pump2OverrideTimeEpochEnd != 0)
    {
      // timer elapsed, back to auto
      pump2OverrideTimeEpochEnd = 0;
      pump2Override = false;
      // Send Events to the Web Client with the Sensor Readings
      String status = "Status: " + (pump2Status) ? "On " : "Off ";
      status += "(Auto)";
      events.send(status.c_str(), "pump2Status", millis());
    }
  }
}