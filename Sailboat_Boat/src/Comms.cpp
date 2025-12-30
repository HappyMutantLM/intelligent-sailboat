#include "Comms.h"

CommsManager::CommsManager() : radio(new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1)) {
    _lastRudder = 0;
    _lastSail = 50;  // Center position
    _currentMode = MODE_MANUAL;
    _waypointLat = 0.0;
    _waypointLon = 0.0;
}

bool CommsManager::begin() {
    // Initialize LoRa at 915.0 MHz
    int state = radio.begin(915.0);
    if (state == RADIOLIB_ERR_NONE) {
        radio.setOutputPower(20); // Max power (20dBm) for long range
        
        // Optional: Configure for better range
        radio.setSpreadingFactor(10);  // Higher = longer range, slower data
        radio.setBandwidth(125.0);     // 125 kHz bandwidth
        radio.setCodingRate(8);        // 4/8 coding rate
        
        Serial.println("LoRa initialized successfully");
        return true;
    }
    
    Serial.print("LoRa init failed, code: ");
    Serial.println(state);
    return false;
}

void CommsManager::sendTelemetry(float lat, float lon, float heading, float wind, float battery) {
    TelemetryPacket packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.heading = heading;
    packet.windAngle = wind;
    packet.battery = battery;
    packet.updateChecksum();
    
    int state = radio.transmit((uint8_t*)&packet, sizeof(packet));
    
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("Telemetry TX failed: ");
        Serial.println(state);
    } else {
        Serial.println("Telemetry sent");
    }
}

PacketType CommsManager::checkReceive() {
    // Non-blocking check for incoming data
    int state = radio.receive(_rxBuffer, sizeof(_rxBuffer));
    
    if (state == RADIOLIB_ERR_NONE) {
        // We received something!
        PacketType pktType = getPacketType(_rxBuffer);
        
        Serial.print("Received packet type: 0x");
        Serial.println(pktType, HEX);
        
        switch (pktType) {
            case PKT_CONTROL: {
                ControlPacket* ctrl = (ControlPacket*)_rxBuffer;
                if (ctrl->isValid()) {
                    _lastRudder = ctrl->rudder;
                    _lastSail = ctrl->sail;
                    Serial.print("Control: Rudder=");
                    Serial.print(_lastRudder);
                    Serial.print(" Sail=");
                    Serial.println(_lastSail);
                    return PKT_CONTROL;
                } else {
                    Serial.println("Control packet checksum failed!");
                }
                break;
            }
            
            case PKT_WAYPOINT: {
                WaypointPacket* wp = (WaypointPacket*)_rxBuffer;
                if (wp->isValid()) {
                    _waypointLat = wp->targetLat;
                    _waypointLon = wp->targetLon;
                    Serial.print("Waypoint received: ");
                    Serial.print(_waypointLat, 6);
                    Serial.print(", ");
                    Serial.println(_waypointLon, 6);
                    return PKT_WAYPOINT;
                } else {
                    Serial.println("Waypoint packet checksum failed!");
                }
                break;
            }
            
            case PKT_MODE_CHANGE: {
                ModePacket* mode = (ModePacket*)_rxBuffer;
                if (mode->isValid()) {
                    _currentMode = (SailingMode)mode->mode;
                    Serial.print("Mode changed to: ");
                    Serial.println(_currentMode == MODE_AUTONOMOUS ? "AUTONOMOUS" : "MANUAL");
                    return PKT_MODE_CHANGE;
                } else {
                    Serial.println("Mode packet checksum failed!");
                }
                break;
            }
            
            default:
                Serial.print("Unknown packet type: 0x");
                Serial.println(pktType, HEX);
                break;
        }
    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
        // No data available, this is normal
    } else {
        // Some other error
        Serial.print("RX error: ");
        Serial.println(state);
    }
    
    return (PacketType)0; // No valid packet received
}
