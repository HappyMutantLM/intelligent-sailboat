#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <Arduino.h>

// Note: We REMOVED the library include from here to prevent
// "Multiple Definition" linker errors.

// PIN DEFINITIONS
#define PIN_RUDDER_SERVO  A1
#define PIN_SAIL_SERVO    A2

class ActuatorManager {
public:
    ActuatorManager();
    void begin();

    // -100 (Left) to 100 (Right)
    void setRudder(int percentage);

    // 0 (Loose) to 100 (Tight)
    void setSail(int percentage);

private:
    int _rudderIndex;
    int _sailIndex;

    // Calibration: Pulse widths in microseconds
    const int RUDDER_MIN_US = 1000;
    const int RUDDER_MAX_US = 2000;
    
    const int SAIL_LOOSE_US = 1000; 
    const int SAIL_TIGHT_US = 2000;
};

#endif