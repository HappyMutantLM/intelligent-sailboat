#ifndef COMMS_H
#define COMMS_H

#include <Arduino.h>
#include <RadioLib.h>
#include "Protocol.h"  // <-- Use unified protocol

// Define pins for Metro M4 + RFM95 Breakout
// YOU MUST CHECK YOUR PHYSICAL WIRING HERE
#define LORA_CS   10  // Example: Pin 10
#define LORA_DIO0 2   // Example: Pin 2 (Interrupt)
#define LORA_RST  9   // Example: Pin 9
#define LORA_DIO1 3   // Optional

class CommsManager {
public:
    CommsManager();
    bool begin();
    
    // Send telemetry packet (Using unified protocol)
    void sendTelemetry(float lat, float lon, float heading, float wind, float battery);
    
    // Check for incoming commands (returns packet type, or 0 if none)
    PacketType checkReceive();
    
    // Getters for latest received data
    int16_t getManualRudder() { return _lastRudder; }
    int16_t getManualSail() { return _lastSail; }
    SailingMode getMode() { return _currentMode; }
    float getWaypointLat() { return _waypointLat; }
    float getWaypointLon() { return _waypointLon; }
    
    // Check if we're in autonomous mode
    bool isAutonomous() { return _currentMode == MODE_AUTONOMOUS; }

private:
    SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
    
    // Received data storage
    int16_t _lastRudder;
    int16_t _lastSail;
    SailingMode _currentMode;
    float _waypointLat;
    float _waypointLon;
    
    // Receive buffer
    uint8_t _rxBuffer[256];
};

#endif
