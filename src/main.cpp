// Source code adapted from: (was originally written for ESP8266):
// https://lastminuteengineers.com/esp8266-ntp-server-date-time-tutorial/

// IMPORTANT: install NTPclient library from Fabrice Weinberg in order to run this code (available in the Arduino IDE under Sketch -> Include Library -> Manage Libraries
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ESP32Time.h>
#include <DHT.h>

#define UTC_OFFSET_IN_SECONDS -36000 // offset from greenwich time (Hawaii is UTC-10)
#define NTP_SYNC_HOUR 4
#define NTP_SYNC_MINUTE 0
#define NTP_SYNC_SECOND 0

// pin definitons
#define LED_PIN 2
#define WATER_PUMP_1_PIN 22
#define DHT_PIN 23

// function declarations
void updateAndSyncTime();
String processor(const String &var);

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_IN_SECONDS);
ESP32Time rtc; // no offset, as that is already added from NTPClient
bool rtcUpdated = false;

// time interval setup
int interval = 10000;
unsigned long previousMillis = 0;
unsigned long now;

// SSID and password of Wifi connection:
const char *ssid = "Beast";
const char *password = "ca9786e7";

AsyncWebServer server(80);
DHT dht(DHT_PIN, DHT11);

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String ledState;
bool pumpOn = true;

void setup()
{
  Serial.begin(115200);
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

  WiFi.mode(WIFI_STA); // station mode: ESP32 connects to access point
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP()); // send ip address of esp32

  // mdns responder for esp32.local
  if (MDNS.begin("esp32"))
  {
    Serial.println("MDNS responder started, accessible via esp32.local");
  }

  timeClient.begin();

  // The function timeClient.update() syncs the local time to the NTP server. In the video I call this in the main loop. However, NTP servers dont like it if
  // they get pinged all the time, so I recommend to only re-sync to the NTP server occasionally. In this example code we only call this function once in the
  // setup() and you will see that in the loop the local time is automatically updated. Of course the ESP/Arduino does not have an infinitely accurate clock,
  // so if the exact time is very important you will need to re-sync once in a while.
  updateAndSyncTime();

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", String(), false, processor); });

  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/style.css", "text/css"); });

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

  server.begin();
}

void loop()
{
  now = millis();
  if (now - previousMillis > interval)
  {
    // Serial.println(rtc.getTime());
    // get dht sensor readings
    float h = dht.readHumidity();
    float f = dht.readTemperature(true); // true outputs in fahrenheit
    if (isnan(h) || isnan(f))
    {
      Serial.println("Error: Failed to read from DHT sensor!");
    }
    else
    {
      // Compute heat index in Fahrenheit
      float hif = dht.computeHeatIndex(f, h);
      Serial.println((String) "Temperature: " + f + "F");
      Serial.println((String) "Humidity: " + h + "%");
      Serial.println((String) "Heat Index: " + hif + "F");
    }

    previousMillis += interval;
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

  // Run water pump 1 from 6am to 6pm continuously.  The other 12 hours, the pump will run for 1 min on the hour
  /*if (currentHour >= 6 and currentHour <= 18)
  {
    digitalWrite(WATER_PUMP_1_PIN, HIGH);
  }
  else if (currentMin == 0)
  {
    digitalWrite(WATER_PUMP_1_PIN, HIGH);
  }
  else
  {
    digitalWrite(WATER_PUMP_1_PIN, LOW);
  }
  */
}

void updateAndSyncTime()
{
  if (timeClient.update())
  {
    // successful update
    Serial.println("Recieved updated time from NTP!");
    rtc.setTime(timeClient.getEpochTime());
    rtcUpdated = true;
  }
  else
  {
    Serial.println("Unable to connect to NTP");
  }
}
String processor(const String &var)
{
  // Serial.println(var);
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
    return rtc.getTime("%A, %B %d %Y %r");
  }
  return String();
}