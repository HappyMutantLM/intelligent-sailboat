#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>

enum SailMode {
    MODE_MANUAL,
    MODE_AUTONOMOUS
};

struct NavState {
    float currentLat;
    float currentLon;
    float currentHeading; // Compass (0-360)
    float windAngle;      // Apparent wind relative to boat (0 = Bow, 180 = Stern)
};

class Navigator {
public:
    Navigator();
    
    void setTarget(float lat, float lon);
    void setMode(SailMode mode);
    
    // The main calculation loop
    void update(NavState state);

    // Getters for the Actuators to read
    int getDesiredRudder(); // -100 to 100
    int getDesiredSail();   // 0 to 100

private:
    SailMode _mode;
    float _targetLat, _targetLon;
    
    int _outputRudder;
    int _outputSail;

    // Helpers
    float getBearingToTarget(float lat1, float lon1, float lat2, float lon2);
    float getDistanceToTarget(float lat1, float lon1, float lat2, float lon2);
    float normalizeAngle(float angle);
};

#endif