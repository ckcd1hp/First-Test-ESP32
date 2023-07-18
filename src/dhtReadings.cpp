#include <ESPAsyncWebServer.h>

#include "dhtReadings.h"

DHT dht(DHT_PIN, DHT11);
extern AsyncEventSource events;

void getDhtReadings()
{
    float h, f, hif; // humidity, temp in fahrenheit, heat index fahrenheit
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
        // Serial.println((String) "Temperature: " + f + "F");
        // Serial.println((String) "Humidity: " + h + "%");
        // Serial.println((String) "Heat Index: " + hif + "F");
        // Serial.println(rtc.getTime());
        // Send Events to the Web Client with the Sensor Readings
        events.send(String(f).c_str(), "temperature", millis());
        events.send(String(h).c_str(), "humidity", millis());
        events.send(String(hif).c_str(), "heatIndex", millis());
    }
}