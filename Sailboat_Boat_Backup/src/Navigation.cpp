#include "Navigation.h"

NavigationManager::NavigationManager() {
    _kp = 1.0; // Simple P-Controller gain
}

int NavigationManager::update(float currentHeading, float targetHeading) {
    // 1. Calculate Error (shortest path)
    float error = targetHeading - currentHeading;
    
    // Handle wrap-around
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    // 2. P-Controller
    int rudderAngle = (int)(error * _kp);
    
    // 3. Limit Rudder throw
    return constrain(rudderAngle, -45, 45); 
}