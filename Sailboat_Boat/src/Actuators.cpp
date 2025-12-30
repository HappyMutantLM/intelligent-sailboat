#include "Actuators.h"
#include "SAMD_ISR_Servo.h" // <--- The include moves HERE

// Define the Global ISR Servo Object locally in this file
SAMD_ISR_Servo isrServo;

ActuatorManager::ActuatorManager() {
    _rudderIndex = -1;
    _sailIndex = -1;
}

void ActuatorManager::begin() {
    // Setup Rudder (Returns an index, e.g., 0)
    _rudderIndex = isrServo.setupServo(PIN_RUDDER_SERVO, RUDDER_MIN_US, RUDDER_MAX_US);
    
    // Setup Sail (Returns an index, e.g., 1)
    _sailIndex = isrServo.setupServo(PIN_SAIL_SERVO, SAIL_LOOSE_US, SAIL_TIGHT_US);

    if (_rudderIndex == -1 || _sailIndex == -1) {
        Serial.println("Error: Could not setup servos!");
    }

    // Safe start
    setRudder(0); 
    setSail(0);   
}

void ActuatorManager::setRudder(int percentage) {
    if (_rudderIndex == -1) return;

    percentage = constrain(percentage, -100, 100);
    // Map -100/100 to 0-180 degrees
    int degrees = map(percentage, -100, 100, 0, 180);
    
    isrServo.setPosition(_rudderIndex, degrees);
}

void ActuatorManager::setSail(int percentage) {
    if (_sailIndex == -1) return;

    percentage = constrain(percentage, 0, 100);
    int degrees = map(percentage, 0, 100, 0, 180);
    
    isrServo.setPosition(_sailIndex, degrees);
}