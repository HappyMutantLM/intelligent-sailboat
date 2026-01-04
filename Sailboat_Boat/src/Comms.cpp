#include "Comms.h"

CommsManager::CommsManager() : radio(new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1)) {
    _lastRudder = 0;
    _lastSail = 0;
    _autonomousMode = false;
}

bool CommsManager::begin() {
    // Initialize LoRa at 915.0 MHz
    int state = radio.begin(915.0);
    if (state == RADIOLIB_ERR_NONE) {
        radio.setOutputPower(20); // Max power (20dBm) for long range
        return true;
    }
    return false;
}

void CommsManager::sendTelemetry(float lat, float lon, float heading, float wind, float battery) {
    // Create a compact binary packet (better than String for range/speed)
    struct TelemetryPacket {
        float lat, lon, head, wind, batt;
    } packet;

    packet.lat = lat;
    packet.lon = lon;
    packet.head = heading;
    packet.wind = wind;
    packet.batt = battery;

    radio.transmit((uint8_t*)&packet, sizeof(packet));
}

bool CommsManager::checkReceive() {
    // Simple polling check
    // In a real app, you might use interrupts
    int len = radio.getPacketLength();
    if (len > 0) {
        uint8_t buffer[256];
        int state = radio.receive(buffer, len);
        
        if (state == RADIOLIB_ERR_NONE) {
            // Parse Command Packet (Define your protocol here)
            // Example: [Type, Rudder_High, Rudder_Low, Sail_High, Sail_Low]
            if (buffer[0] == 1) { // 1 = Control Packet
                _lastRudder = (int16_t)((buffer[1] << 8) | buffer[2]);
                _lastSail =   (int16_t)((buffer[3] << 8) | buffer[4]);
                return true;
            }
        }
    }
    return false;
}
