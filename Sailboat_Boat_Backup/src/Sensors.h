#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Adafruit_GPS.h>

class SensorManager {
public:
    SensorManager();
    bool begin();
    void update();

    // Getters
    float getHeading();       // Returns GPS Course (0-360)
    float getRudderAngle();   // Returns Encoder Angle (0-360)
    float getSailAngle();     // Returns Encoder Angle (0-360)
    float getBatteryVoltage();
    float getLat();
    float getLon();

private:
    Adafruit_GPS _gps;
    float _currentHeading;
};

#endif