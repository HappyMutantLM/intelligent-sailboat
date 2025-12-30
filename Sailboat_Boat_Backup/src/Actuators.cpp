#include "Actuators.h"

// --- PIN DEFINITIONS ---
#define RUDDER_PIN 10  // Confirmed by user
#define SAIL_PIN   11  // Confirmed by user

void ActuatorManager::begin() {
    _rudderServo.attach(RUDDER_PIN);
    _sailServo.attach(SAIL_PIN);
    
    // --- WIGGLE TEST (Startup Routine) ---
    // This confirms wiring is correct before the radio takes over.
    Serial.println("   [TEST] Wiggle Routine Starting...");
    
    // 1. Rudder Test (Left -> Right -> Center)
    _rudderServo.write(45);  // Left
    delay(500);
    _rudderServo.write(135); // Right
    delay(500);
    _rudderServo.write(90);  // Center
    delay(200);

    // 2. Sail Test (Tight -> Loose -> Safe)
    _sailServo.write(0);     // Tight
    delay(500);
    _sailServo.write(90);    // Loose
    delay(500);
    _sailServo.write(45);    // Halfway (Safe Mode)
    
    Serial.println("   [TEST] Wiggle Routine Complete.");
}

void ActuatorManager::setRudder(int angle) {
    // Input: -45 (Left) to +45 (Right)
    // Servo: 45 (Left) to 135 (Right)
    int servoValue = map(angle, -45, 45, 45, 135); 
    servoValue = constrain(servoValue, 0, 180);
    _rudderServo.write(servoValue);
}

void ActuatorManager::setSail(int angle) {
    // Input: 0 (Tight) to 90 (Loose)
    // Servo: 0 (Tight) to 120 (Loose)
    int servoValue = map(angle, 0, 90, 0, 120);
    servoValue = constrain(servoValue, 0, 180);
    _sailServo.write(servoValue);
}