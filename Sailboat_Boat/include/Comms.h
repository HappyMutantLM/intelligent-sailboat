#ifndef COMMS_H
#define COMMS_H

#include <Arduino.h>
#include <RadioLib.h>

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
    
    // Send telemetry packet (Lat, Lon, Heading, etc.)
    void sendTelemetry(float lat, float lon, float heading, float wind, float battery);
    
    // Check for incoming commands
    bool checkReceive();
    
    // Getters for latest commands
    int16_t getManualRudder() { return _lastRudder; }
    int16_t getManualSail() { return _lastSail; }
    bool isAutonomous() { return _autonomousMode; }

private:
    SX1276 radio;
    
    int16_t _lastRudder;
    int16_t _lastSail;
    bool _autonomousMode;
};

#endif
