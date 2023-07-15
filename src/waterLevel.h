#include "pins.h"

#ifndef HC_SR04_H
#define HC_SR04_H

#define CM true
#define INCH false

enum WaterLevel
{
    W_LOW,
    W_MED,
    W_HIGH
};

void checkWaterLevel();                 // start ultrasonic sensor to check water level
void displayWaterLevel(int distanceCm); // display water level on web page

class HC_SR04
{
public:
    HC_SR04(int trigger, int echo, int interrupt, int max_dist = 200);

    void begin();
    void start();
    bool isFinished() { return _finished; }
    unsigned int getRange(bool units = CM);
    static HC_SR04 *instance() { return _instance; }

private:
    static void _echo_isr();

    int _trigger, _echo, _int, _max_dist;
    volatile unsigned long _start, _end;
    volatile bool _finished;
    static HC_SR04 *_instance;
};

extern HC_SR04 ultrasonicSensor;

#endif