#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>

class NavigationManager {
public:
    NavigationManager();
    
    // Calculates rudder angle to hit target
    int update(float currentHeading, float targetHeading);

private:
    float _kp; // Proportional Gain
};

#endif