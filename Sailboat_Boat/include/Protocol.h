/*
 * Protocol.h
 * 
 * Unified Communication Protocol for Intelligent Sailboat
 * This file should be copied to BOTH the Boat and Controller projects
 * 
 * Author: Leila
 * Description: Defines all LoRa packet structures and helper functions
 *              for communication between Boat, Controller, and Base Station
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

// ============================================================================
// PACKET TYPE DEFINITIONS
// ============================================================================
enum PacketType : uint8_t {
    PKT_CONTROL     = 0x01,  // Controller -> Boat: Manual control commands
    PKT_TELEMETRY   = 0x02,  // Boat -> Controller/Base: Status update
    PKT_WAYPOINT    = 0x03,  // Base/Controller -> Boat: Set destination
    PKT_MODE_CHANGE = 0x04,  // Controller/Base -> Boat: Manual/Auto switch
    PKT_ACK         = 0x05,  // Any -> Any: Acknowledgment (future use)
    PKT_HEARTBEAT   = 0x06   // Boat -> Base: Keep-alive ping (future use)
};

// ============================================================================
// SAILING MODES
// ============================================================================
enum SailingMode : uint8_t {
    MODE_MANUAL     = 0x00,  // RC Control via joystick
    MODE_AUTONOMOUS = 0x01   // Navigate to waypoint
};

// ============================================================================
// PACKET STRUCTURES
// ============================================================================

/**
 * ControlPacket
 * Sent from Controller to Boat for manual RC operation
 * 
 * Payload: 6 bytes
 * Frequency: ~10 Hz (every 100ms) when controller is active
 */
struct ControlPacket {
    uint8_t  type;      // Must be PKT_CONTROL (0x01)
    int16_t  rudder;    // -100 (full left) to +100 (full right)
    int16_t  sail;      // 0 (tight/close-hauled) to 100 (loose/running)
    uint8_t  checksum;  // Simple XOR checksum
    
    // Constructor
    ControlPacket() : type(PKT_CONTROL), rudder(0), sail(50), checksum(0) {}
    
    // Calculate checksum
    void updateChecksum() {
        checksum = type ^ (rudder >> 8) ^ (rudder & 0xFF) ^ (sail >> 8) ^ (sail & 0xFF);
    }
    
    // Verify checksum
    bool isValid() const {
        uint8_t calc = type ^ (rudder >> 8) ^ (rudder & 0xFF) ^ (sail >> 8) ^ (sail & 0xFF);
        return calc == checksum;
    }
} __attribute__((packed));

/**
 * TelemetryPacket
 * Sent from Boat to Controller/Base Station
 * 
 * Payload: 22 bytes
 * Frequency: ~1 Hz (every 1000ms)
 */
struct TelemetryPacket {
    uint8_t  type;      // Must be PKT_TELEMETRY (0x02)
    float    lat;       // Latitude (degrees)
    float    lon;       // Longitude (degrees)
    float    heading;   // Compass heading 0-360 (degrees, 0=North)
    float    windAngle; // Apparent wind angle 0-360 (degrees, 0=Bow, 180=Stern)
    float    battery;   // Battery voltage (volts)
    uint8_t  checksum;  // Simple XOR checksum
    
    // Constructor
    TelemetryPacket() : type(PKT_TELEMETRY), lat(0), lon(0), heading(0), 
                        windAngle(0), battery(0), checksum(0) {}
    
    // Calculate checksum
    void updateChecksum() {
        // XOR all the bytes together (simplified checksum)
        uint8_t *bytes = (uint8_t*)this;
        checksum = 0;
        for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) {
            checksum ^= bytes[i];
        }
    }
    
    // Verify checksum
    bool isValid() const {
        uint8_t *bytes = (uint8_t*)this;
        uint8_t calc = 0;
        for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) {
            calc ^= bytes[i];
        }
        return calc == checksum;
    }
} __attribute__((packed));

/**
 * WaypointPacket
 * Sent from Base Station or Controller to Boat
 * 
 * Payload: 10 bytes
 * Frequency: On-demand (when user sets new waypoint)
 */
struct WaypointPacket {
    uint8_t  type;       // Must be PKT_WAYPOINT (0x03)
    float    targetLat;  // Target latitude (degrees)
    float    targetLon;  // Target longitude (degrees)
    uint8_t  checksum;   // Simple XOR checksum
    
    // Constructor
    WaypointPacket() : type(PKT_WAYPOINT), targetLat(0), targetLon(0), checksum(0) {}
    
    // Calculate checksum
    void updateChecksum() {
        uint8_t *bytes = (uint8_t*)this;
        checksum = 0;
        for (size_t i = 0; i < sizeof(WaypointPacket) - 1; i++) {
            checksum ^= bytes[i];
        }
    }
    
    // Verify checksum
    bool isValid() const {
        uint8_t *bytes = (uint8_t*)this;
        uint8_t calc = 0;
        for (size_t i = 0; i < sizeof(WaypointPacket) - 1; i++) {
            calc ^= bytes[i];
        }
        return calc == checksum;
    }
} __attribute__((packed));

/**
 * ModePacket
 * Sent from Controller or Base to Boat
 * 
 * Payload: 3 bytes
 * Frequency: On-demand (when user changes mode)
 */
struct ModePacket {
    uint8_t  type;      // Must be PKT_MODE_CHANGE (0x04)
    uint8_t  mode;      // SailingMode enum (0=Manual, 1=Autonomous)
    uint8_t  checksum;  // Simple XOR checksum
    
    // Constructor
    ModePacket() : type(PKT_MODE_CHANGE), mode(MODE_MANUAL), checksum(0) {}
    
    // Calculate checksum
    void updateChecksum() {
        checksum = type ^ mode;
    }
    
    // Verify checksum
    bool isValid() const {
        return (type ^ mode) == checksum;
    }
} __attribute__((packed));

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/**
 * getPacketType
 * Peek at the first byte to determine packet type
 * 
 * @param buffer Pointer to received data
 * @return PacketType enum value
 */
inline PacketType getPacketType(const uint8_t* buffer) {
    return (PacketType)buffer[0];
}

/**
 * printPacketInfo
 * Debug helper - prints packet contents to Serial
 * 
 * @param pkt Pointer to any packet structure
 * @param size Size of the packet
 */
inline void printPacketInfo(const void* pkt, size_t size) {
    const uint8_t* bytes = (const uint8_t*)pkt;
    Serial.print("PKT[");
    Serial.print(size);
    Serial.print("]: ");
    for (size_t i = 0; i < size; i++) {
        if (bytes[i] < 0x10) Serial.print("0");
        Serial.print(bytes[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

/**
 * mapJoystickToRudder
 * Converts joystick X-axis reading to rudder percentage
 * Includes deadzone handling and split mapping
 * 
 * @param rawValue Raw ADC reading (0-1023 typical)
 * @param center Calibrated center point
 * @param min Calibrated minimum value
 * @param max Calibrated maximum value
 * @param deadzone Size of center deadzone
 * @return Rudder value -100 to +100
 */
inline int16_t mapJoystickToRudder(int rawValue, int center, int min, int max, int deadzone) {
    if (rawValue < center - deadzone) {
        // Left side
        return map(rawValue, min, center - deadzone, -100, 0);
    } else if (rawValue > center + deadzone) {
        // Right side
        return map(rawValue, center + deadzone, max, 0, 100);
    } else {
        // Deadzone
        return 0;
    }
}

/**
 * mapJoystickToSail
 * Converts joystick Y-axis reading to sail percentage
 * Includes deadzone handling and split mapping
 * 
 * @param rawValue Raw ADC reading (0-1023 typical)
 * @param center Calibrated center point
 * @param min Calibrated minimum value
 * @param max Calibrated maximum value
 * @param deadzone Size of center deadzone
 * @return Sail value 0 (tight) to 100 (loose)
 */
inline int16_t mapJoystickToSail(int rawValue, int center, int min, int max, int deadzone) {
    if (rawValue < center - deadzone) {
        // Up/Tight
        return map(rawValue, min, center - deadzone, 0, 50);
    } else if (rawValue > center + deadzone) {
        // Down/Loose
        return map(rawValue, center + deadzone, max, 50, 100);
    } else {
        // Center point
        return 50;
    }
}

// ============================================================================
// PROTOCOL VERSION
// ============================================================================
#define PROTOCOL_VERSION "1.0.0"
#define PROTOCOL_DATE "2024-12-30"

#endif // PROTOCOL_H
