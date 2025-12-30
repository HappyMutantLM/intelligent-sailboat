#include "Navigation.h"
#include <math.h>

Navigator::Navigator() {
    _mode = MODE_MANUAL;
    _outputRudder = 0;
    _outputSail = 0;
}

void Navigator::setTarget(float lat, float lon) {
    _targetLat = lat;
    _targetLon = lon;
}

void Navigator::setMode(SailMode mode) {
    _mode = mode;
}

int Navigator::getDesiredRudder() { return _outputRudder; }
int Navigator::getDesiredSail()   { return _outputSail; }

void Navigator::update(NavState state) {
    if (_mode == MODE_MANUAL) return; // Do nothing, let RC control override

    // 1. Calculate Course to Waypoint
    float bearingToTarget = getBearingToTarget(state.currentLat, state.currentLon, _targetLat, _targetLon);
    
    // 2. Calculate Heading Error (How far off are we?)
    float headingError = bearingToTarget - state.currentHeading;
    // Normalize to -180 to 180
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;

    // 3. WINCH LOGIC: Automatic Sail Trim
    // If wind is at 0 (nose), sails in (100%). If wind is 180 (tail), sails out (0%).
    _outputSail = map(abs((int)state.windAngle), 0, 180, 100, 0);

    // 4. RUDDER LOGIC: The "Intelligent" Part
    // Check if the target is in the "No Go Zone" (approx 45 degrees off the bow)
    float windRelToTarget = abs(state.windAngle); // Simplified logic
    
    if (windRelToTarget < 45.0) {
        // TACKING LOGIC
        // We cannot sail straight to target. We must tack.
        // If the target is slightly to our right, we tack 45 deg right.
        if (headingError > 0) _outputRudder = 45; 
        else _outputRudder = -45;
    } else {
        // STANDARD SAILING (PID Control)
        // Simple P-Controller: Turn proportional to error
        // Gain of 1.5 means 10 degree error = 15% rudder
        _outputRudder = constrain((int)(headingError * 1.5), -100, 100);
    }
}

// Haversine / Bearing Helpers
float Navigator::getBearingToTarget(float lat1, float lon1, float lat2, float lon2) {
    // Convert to radians
    float phi1 = radians(lat1);
    float phi2 = radians(lat2);
    float lam1 = radians(lon1);
    float lam2 = radians(lon2);

    float y = sin(lam2 - lam1) * cos(phi2);
    float x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(lam2 - lam1);
    float theta = atan2(y, x);
    
    return degrees(theta);
}