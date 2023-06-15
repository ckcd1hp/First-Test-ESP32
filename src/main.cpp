// Source code adapted from: (was originally written for ESP8266):
// https://lastminuteengineers.com/esp8266-ntp-server-date-time-tutorial/

// IMPORTANT: install NTPclient library from Fabrice Weinberg in order to run this code (available in the Arduino IDE under Sketch -> Include Library -> Manage Libraries
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>

#define UTC_OFFSET_IN_SECONDS -36000 // offset from greenwich time (Hawaii is UTC-10)

String getTimeStampString();

// SSID and password of Wifi connection:
const char *ssid = "Beast";
const char *password = "ca9786e7";

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
String webpage = "<!DOCTYPE html><html><head><title>Page Title</title></head><body style='background-color: #EEEEEE;'><span style='color: #003366;'><h1>This is a Heading</h1><p>This is a paragraph.</p></span></body></html>";

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_IN_SECONDS);

// time interval setup
int interval = 10000;
unsigned long previousMillis = 0;
unsigned long now;

void setup()
{
  Serial.begin(115200);
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
  timeClient.update();
  server.on("/", []()
            { server.send(200, "text/html", webpage); });
  server.begin();
}

void loop()
{
  server.handleClient();

  now = millis();
  if (now - previousMillis > interval)
  {
    // Option1: Get time and day of the week directly (discussed in Youtube video)
    Serial.print("Option 1: ");
    Serial.print(daysOfTheWeek[timeClient.getDay()]);
    Serial.print(", ");
    Serial.print(timeClient.getHours());
    Serial.print(":");
    Serial.print(timeClient.getMinutes());
    Serial.print(":");
    Serial.println(timeClient.getSeconds());

    // Option 2: abstract from formatted date (added later as per request)
    Serial.print("Option 2: ");
    Serial.println(getTimeStampString());
    previousMillis += interval;
  }
}

String getTimeStampString()
{
  time_t rawtime = timeClient.getEpochTime();
  struct tm *ti;
  ti = localtime(&rawtime);

  uint16_t year = ti->tm_year + 1900;
  String yearStr = String(year);

  uint8_t month = ti->tm_mon + 1;
  String monthStr = month < 10 ? "0" + String(month) : String(month);

  uint8_t day = ti->tm_mday;
  String dayStr = day < 10 ? "0" + String(day) : String(day);

  uint8_t hours = ti->tm_hour;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);

  uint8_t minutes = ti->tm_min;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  uint8_t seconds = ti->tm_sec;
  String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);

  return yearStr + "-" + monthStr + "-" + dayStr + " " +
         hoursStr + ":" + minuteStr + ":" + secondStr;
}