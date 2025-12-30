#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // <--- ADD THIS LINE to force the library load
#include "Sensors.h"
#include "Comms.h"
#include "Actuators.h"
#include "Navigation.h"

// -- GLOBAL OBJECTS --
SensorManager   sensors;
CommsManager    comms;
ActuatorManager actuators;
Navigator       navigator;

// Status LED (Built-in NeoPixel on Metro M4)
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel statusLED(1, 40, NEO_GRB + NEO_KHZ800); // Pin 40 is standard for Metro M4 NeoPixel

// Timing Helpers
uint32_t lastTelemetryTime = 0;

void setup() {
    Serial.begin(115200);
    statusLED.begin();
    statusLED.setBrightness(20);
    statusLED.setPixelColor(0, 255, 0, 0); // Red = Booting
    statusLED.show();

    // 1. Init Sensors
    Serial.println("Init Sensors...");
    if (!sensors.begin()) {
        Serial.println("Sensor Failure! Halting.");
        while(1) { // Flash Red/Off on error
            statusLED.setPixelColor(0, 255, 0, 0); statusLED.show(); delay(200);
            statusLED.setPixelColor(0, 0, 0, 0); statusLED.show(); delay(200);
        }
    }

    // 2. Init Comms
    Serial.println("Init LoRa...");
    if (!comms.begin()) {
        Serial.println("LoRa Failure!");
    }

    // 3. Init Actuators
    actuators.begin();
    
    // 4. Calibration (User should verify these!)
    // Assuming you align everything to "0" mechanically before boot
    sensors.setRudderOffset(0.0);
    sensors.setWindOffset(0.0);

    statusLED.setPixelColor(0, 0, 255, 0); // Green = Ready
    statusLED.show();
    Serial.println("Boat Ready.");
}

void loop() {
    // 1. UPDATE SENSORS (Must be fast!)
    sensors.update();

    // 2. CHECK COMMS (Incoming commands?)
    if (comms.checkReceive()) {
        // Flash Blue when receiving
        statusLED.setPixelColor(0, 0, 0, 255); statusLED.show();
        
        if (comms.isAutonomous()) {
            navigator.setMode(MODE_AUTONOMOUS);
            // If the packet contained a waypoint, Navigator handles it
        } else {
            navigator.setMode(MODE_MANUAL);
            actuators.setRudder(comms.getManualRudder());
            actuators.setSail(comms.getManualSail());
        }
    }

    // 3. AUTONOMOUS NAVIGATION LOGIC
    // Create the state object for the brain
    NavState currentState;
    currentState.currentLat = sensors.getLatitude();
    currentState.currentLon = sensors.getLongitude();
    currentState.currentHeading = sensors.getHeading();
    currentState.windAngle = sensors.getWindAngle();

    // Ask the Navigator what to do
    navigator.update(currentState);

    // If we are in Auto mode, apply the calculated values
    // (If Manual, the comms block above already handled it)
    if (comms.isAutonomous()) {
        actuators.setRudder(navigator.getDesiredRudder());
        actuators.setSail(navigator.getDesiredSail());
    }

    // 4. TELEMETRY (Send status back to base every 1 second)
    if (millis() - lastTelemetryTime > 1000) {
        lastTelemetryTime = millis();
        
        // Return LED to Green (or Yellow if no GPS)
        if (sensors.hasGPSFix()) statusLED.setPixelColor(0, 0, 255, 0);
        else statusLED.setPixelColor(0, 255, 200, 0); // Orange = No GPS
        statusLED.show();

        comms.sendTelemetry(
            currentState.currentLat,
            currentState.currentLon,
            currentState.currentHeading,
            currentState.windAngle,
            0.0 // Todo: Add battery voltage reading on A0?
        );
        
        Serial.print("Head: "); Serial.print(currentState.currentHeading);
        Serial.print(" Wind: "); Serial.println(currentState.windAngle);
    }
}