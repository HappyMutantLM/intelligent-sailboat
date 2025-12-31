/*
 * Intelligent Sailboat - Base Station LoRa Dongle
 * Feather M0 with RFM95 LoRa Radio
 * 
 * Acts as USB-Serial bridge between Raspberry Pi and Boat
 * Translates between our binary protocol and RadioHead packets
 */

#include <SPI.h>
#include <RH_RF95.h>

// Pin definitions for Feather M0 RFM95
#define RFM95_CS    8
#define RFM95_RST   4
#define RFM95_INT   3
#define RF95_FREQ   915.0

// LED for visual feedback
#define LED_PIN     13

// RadioHead radio object
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// === PROTOCOL DEFINITIONS (Match Protocol.h) ===
enum PacketType : uint8_t {
    PKT_CONTROL     = 0x01,
    PKT_TELEMETRY   = 0x02,
    PKT_WAYPOINT    = 0x03,
    PKT_MODE_CHANGE = 0x04,
};

struct TelemetryPacket {
    uint8_t  type;
    float    lat;
    float    lon;
    float    heading;
    float    windAngle;
    float    battery;
    uint8_t  checksum;
} __attribute__((packed));

struct WaypointPacket {
    uint8_t  type;
    float    targetLat;
    float    targetLon;
    uint8_t  checksum;
} __attribute__((packed));

struct ControlPacket {
    uint8_t  type;
    int16_t  rudder;
    int16_t  sail;
    uint8_t  checksum;
} __attribute__((packed));

struct ModePacket {
    uint8_t  type;
    uint8_t  mode;
    uint8_t  checksum;
} __attribute__((packed));

// === STATISTICS ===
uint32_t packetsReceived = 0;
uint32_t packetsSent = 0;
uint32_t lastHeartbeat = 0;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    
    // Start serial connection to Pi
    Serial.begin(115200);
    while (!Serial) delay(10); // Wait for Pi
    
    Serial.println("Base Station LoRa Dongle Starting...");
    
    // Reset LoRa module
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);
    
    // Initialize RadioHead
    if (!rf95.init()) {
        Serial.println("ERROR: LoRa init failed!");
        while (1) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
    }
    
    // Configure for long range (match boat settings)
    rf95.setFrequency(RF95_FREQ);
    rf95.setTxPower(23, false);  // Max power
    
    // Optional: Match boat's RadioLib settings as close as possible
    // rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096); // Long range
    
    Serial.println("LoRa initialized successfully");
    Serial.println("Ready for commands...");
    
    blinkLED(3, 100); // Success indicator
}

void loop() {
    // === 1. CHECK FOR LORA PACKETS FROM BOAT ===
    if (rf95.available()) {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        
        if (rf95.recv(buf, &len)) {
            digitalWrite(LED_PIN, HIGH);
            
            // Forward binary packet to Pi via Serial
            Serial.write(buf, len);
            
            // Append RSSI metadata (simple text after binary)
            Serial.print("\nRSSI:");
            Serial.println(rf95.lastRssi(), DEC);
            
            packetsReceived++;
            digitalWrite(LED_PIN, LOW);
        }
    }
    
    // === 2. CHECK FOR COMMANDS FROM PI ===
    if (Serial.available()) {
        // Read packet type first
        uint8_t pktType = Serial.peek();
        
        // Determine packet size based on type
        size_t expectedSize = 0;
        
        switch (pktType) {
            case PKT_CONTROL:
                expectedSize = sizeof(ControlPacket);
                break;
            case PKT_WAYPOINT:
                expectedSize = sizeof(WaypointPacket);
                break;
            case PKT_MODE_CHANGE:
                expectedSize = sizeof(ModePacket);
                break;
            default:
                // Unknown packet type, flush
                Serial.read();
                continue;
        }
        
        // Wait for complete packet
        if (Serial.available() >= expectedSize) {
            uint8_t txBuf[expectedSize];
            Serial.readBytes(txBuf, expectedSize);
            
            // Transmit to boat via LoRa
            digitalWrite(LED_PIN, HIGH);
            rf95.send(txBuf, expectedSize);
            rf95.waitPacketSent();
            digitalWrite(LED_PIN, LOW);
            
            packetsSent++;
        }
    }
    
    // === 3. HEARTBEAT (Optional) ===
    if (millis() - lastHeartbeat > 5000) {
        lastHeartbeat = millis();
        Serial.print("STATS|RX:");
        Serial.print(packetsReceived);
        Serial.print("|TX:");
        Serial.println(packetsSent);
    }
}

void blinkLED(int times, int delayMs) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(delayMs);
        digitalWrite(LED_PIN, LOW);
        delay(delayMs);
    }
}
