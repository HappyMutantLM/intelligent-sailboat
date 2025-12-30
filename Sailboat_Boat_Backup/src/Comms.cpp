#include "Comms.h"

// --- CONFIGURATION ---
// Radio Pins for Feather M0 with RFM95
#define RADIO_CS 8
#define RADIO_G0 3
#define RADIO_RST 4
#define FREQUENCY 915.0

CommsManager::CommsManager() {
    // Create the Hardware Module and the Radio Driver
    _radio = new RFM95(new Module(RADIO_CS, RADIO_G0, RADIO_RST));
}

bool CommsManager::begin() {
    Serial.print("   ...Starting Radio");
    
    // Initialize the Radio
    int state = _radio->begin(FREQUENCY);
    
    if (state == RADIOLIB_ERR_NONE) {
        _radio->setOutputPower(20); // Set Max Power (20dBm)
        return true;
    }
    
    // Debug Error Code if it fails
    Serial.print(" [ERROR Code: ");
    Serial.print(state);
    Serial.print("] ");
    return false;
}

void CommsManager::sendTelemetry(TelemetryPacket packet) {
    // Format: "HDG,LAT,LON,BATT"
    String data = String(packet.heading, 1) + "," +
                  String(packet.lat, 6) + "," +
                  String(packet.lon, 6) + "," +
                  String(packet.battery, 2);
                  
    _radio->transmit(data);
}

bool CommsManager::receiveControl(ControlPacket &packet) {
    // Check if packet received
    int length = _radio->getPacketLength();
    
    // Safety check: Is the packet the right size?
    if (length == sizeof(ControlPacket)) {
        uint8_t buffer[sizeof(ControlPacket)];
        
        // Read the raw bytes
        int state = _radio->readData(buffer, sizeof(ControlPacket));
        
        if (state == RADIOLIB_ERR_NONE) {
            // Convert raw bytes back into the Struct
            memcpy(&packet, buffer, sizeof(ControlPacket));
            return true;
        }
    }
    return false;
}