# NFT-ESP32
This project uses an ESP32 microcontroller to drive a Nutrient Film Technique (NFT) hydroponics garden.
Features:
1. 2 Water pumps - One pump will run from 6am to 12 pm continuously. The second will run from 12pm to 6pm.  After that, the first pump will run every hour on the hour for 1 minute.  The second pump will run every hour on the half hour for 1 minute.  Statuses are monitored via current sensors, so if one fails, the other will run instead and an alarm will be generated.  Currently, an alarm is generated on the webpage, but will be updated to send a phone notification later.
2. 1 Air pump - Will run on a 24/7 schedule 15 min on, 15 min off.  Also monitored by current sensor and will generate an alarm on the web server.
3. DHT11 Temp and Humidity Sensor - Monitor temp and humidity of nearby area or enclosure temps.  Will generate an alarm on web server for temps above 90F.
4. HC-SR04 Ultrasonic Sensor - Will monitor water levels of reservoir.  Displays low, medium, or high on web server.
5. NTP Sync - connects to pool.ntp.org daily at 4am to sync time to the built in real time clock.
6. Web Server - Accessible via http://esp32.local. Displays last sync time, temp/humidity, water level readings, pump command/status, and alarms.  Offers ability to override pumps for 5-60 minutes
  or permanently.  Also able to set back to auto at any time.

Configuration:
Within the src folder, create a file config.h and create two definitions for your WIFI SSID and password.

#define WIFI_SSID "xxxxxxx"
#define WIFI_PASSWORD "xxxxxx"

Loading code to ESP32
1. Use Visual Studio Code with extension PlatformIO.
2. On the left tab, click on the alien icon.  Under PROJECT TASKS -> esp-wrover-kit -> Platform -> Click Build FileSystem Image.  This flashes the web server files to the SPIFFS (SPI Flash File Storage).
3. Click Upload Filesystem Image
4. View -> Command Palette -> PlatformIO: Upload and Monitor or just PlatformIO: Upload if you don't want to see serial monitor debug statements

Modifications:  This code can be easily modified to suit your purposes!
1. Water pump schedule - Change the function **controlPumps**
2. Air pump schedule - Change the variable **airPumpInterval** (Default is 900000, which is 15 minutes in seconds)
3. Water level calibration - Change function **getWaterLevel** to adjust distances for water level.  By default water level is checked once a minute, when adjusting it'll be easier to speed this up
  via the variable **waterLevelInterval**
5. NTP Sync time - Change definitions **NTP_SYNC_HOUR**, **NTP_SYNC_MINUTE**, and **NTP_SYNC_SECOND** (Default is 4, 0, 0, which is 4am.  Hours is 0-23)
6. Wifi Retry Connection Time - If the ESP32 loses wifi, it will try to reestablish connections every 5 minutes Change definition **WIFI_RETY_WAIT_TIME**
7. Web Server URL - Uses MDNS to access web server at esp32.local as the IP will change.  Under function **WiFiGotIP**, change the string in MDNS.begin("YourNewURL").  You can then access the web server
   via YourNewURL.local

Pins:
Water pump 1 command: 22
Water pump 2 command: 21
Air pump: 19
DHT 11 temp sensor: 23
Water pump 1 status: 34
Water pump 2 status: 35
Air pump status: 32
Ultrasonic sensor trigger: 5
Ultrasonic sensor echo: 18

Required Parts:
1. HC-SR04 Sensor QTY 1
2. DHT11 Sensor QTY 1
3. ESP32 QTY 1
4. ACS712 5A Current Sensor QTY 3
5. 1CH-5V-10 Relay QTY 3
6. 10K Resistor QTY 3  // These resistors are for voltage dividers as the current sensors output 0-5V and the ESP can only read 0-3.3V
7. 5K Resistor QTY 3
