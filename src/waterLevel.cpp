#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "waterLevel.h"

// HC_SR04 *HC_SR04::_instance=NULL;
HC_SR04 *HC_SR04::_instance(NULL);
extern AsyncEventSource events;

// set up ultrasonic sensor for detecting water level on interrupt
HC_SR04 ultrasonicSensor(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, ULTRASONIC_ECHO_PIN);

// what does line mean? HC_SR04::HC_SR04(int trigger, int echo, int interrupt, int max_dist) : _trigger(trigger), _echo(echo), _int(interrupt), _max(max_dist), _finished(false)

HC_SR04::HC_SR04(int trigger, int echo, int interrupt, int max_dist)
    : _trigger(trigger), _echo(echo), _int(interrupt), _max_dist(max_dist), _finished(false)
{
    if (_instance == 0)
        _instance = this;
}

void HC_SR04::begin()
{
    // setup trigger pin as output
    pinMode(_trigger, OUTPUT);
    digitalWrite(_trigger, LOW);
    // setup echo pin as input
    pinMode(_echo, INPUT);
    attachInterrupt(_int, _echo_isr, CHANGE);
}

void HC_SR04::start()
{
    // to start a new measurement, set trigger pin to HIGH for 10us
    _finished = false;
    digitalWrite(_trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigger, LOW);
}

unsigned int HC_SR04::getRange(bool units)
{
    // distance = (time / 2) / 29.1 (or 58) [cm] or (time / 2) / 74 (or 148) [inch]
    return (_end - _start) / ((units) ? 58 : 148);
}
void HC_SR04::_echo_isr()
{
    HC_SR04 *_this = HC_SR04::instance();

    switch (digitalRead(_this->_echo))
    {
    case HIGH:
        _this->_start = micros();
        break;
    case LOW:
        _this->_end = micros();
        _this->_finished = true;
        break;
    }
}
void displayWaterLevel(int distanceCm)
{
    // water level enum
    WaterLevel waterLevel;
    if (distanceCm > 20)
    {
        waterLevel = W_LOW;
    }
    else if (distanceCm > 10)
    {
        waterLevel = W_MED;
    }
    else
    {
        waterLevel = W_HIGH;
    }
    String waterLevelString = (waterLevel == W_LOW) ? "Low" : (waterLevel == W_MED) ? "Medium"
                                                                                    : "High";
    events.send(waterLevelString.c_str(), "waterLevel", millis());
}
void checkWaterLevel()
{
    ultrasonicSensor.start();
}
