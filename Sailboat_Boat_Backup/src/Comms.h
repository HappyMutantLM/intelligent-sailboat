#ifndef COMMS_H
#define COMMS_H

#include <Arduino.h>
#include <RadioLib.h>

// --- SHARED DATA STRUCTURES ---

// 1. Telemetry: Boat -> Remote
struct TelemetryPacket {
    float heading;
    float lat;
    float lon;
    float battery;
};

// 2. Control: Remote -> Boat
struct ControlPacket {
    int16_t rudder; // -45 to +45
    int16_t sail;   // 0 to 90
};

class CommsManager {
public:
    CommsManager();
    bool begin();
    
    // Send data back to the human
    void sendTelemetry(TelemetryPacket packet);
    
    // Check for incoming commands from the human
    bool receiveControl(ControlPacket &packet);

private:
    RFM95* _radio;
};

#endif