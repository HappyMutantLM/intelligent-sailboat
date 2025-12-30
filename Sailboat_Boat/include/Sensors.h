#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BNO08x.h>

// I2C Addresses
#define TCA9548A_ADDR 0x70
#define AS5600_ADDR   0x36

// Multiplexer Channels
#define MUX_CHAN_RUDDER 0
#define MUX_CHAN_SAIL   1
#define MUX_CHAN_WIND   2

class SensorManager {
public:
    SensorManager();

    // Setup hardware
    bool begin();

    // Call this as fast as possible in the main loop!
    // It parses GPS bytes and reads the IMU
    void update();

    // Calibration
    void setRudderOffset(float offsetDeg);
    void setSailOffset(float offsetDeg);
    void setWindOffset(float offsetDeg);

    // Data Getters
    float getRudderAngle();
    float getSailAngle();
    float getWindAngle();
    
    // GPS / Compass Getters
    float getLatitude();
    float getLongitude();
    float getHeading(); // 0-360 degrees (True North if GPS fix, Magnetic otherwise)
    bool  hasGPSFix();

private:
    // Internal Hardware Objects
    Adafruit_GPS _gps;
    Adafruit_BNO08x _bno08x;

    // Data Storage
    float _currentHeading;
    float _rudderOffset;
    float _sailOffset;
    float _windOffset;
    
    // Multiplexer Helper
    void tcaSelect(uint8_t channel);
    float readAS5600();
    float normalizeAngle(float angle);
};

#endif