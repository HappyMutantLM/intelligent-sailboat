#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <RadioLib.h>
#include "ControllerConfig.h"
#include "Protocol.h"  // <-- Use unified protocol

// --- OBJECTS ---
RFM95 radio = new Module(LORA_CS, LORA_IRQ, LORA_RST);
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// --- PROTOCOL PACKETS ---
ControlPacket txControl;
TelemetryPacket lastTelemetry;
ModePacket txMode;

// --- STATE VARIABLES ---
SailingMode currentMode = MODE_MANUAL;
bool linkActive = false;

// --- CALIBRATION VARIABLES ---
int xMin = 0, xMax = 1023, xCenter = 512;
int yMin = 0, yMax = 1023, yCenter = 512;

// --- TIMERS ---
unsigned long lastTx = 0;
unsigned long lastRx = 0;
unsigned long lastUIUpdate = 0;

// --- HELPER: CALIBRATION ROUTINE ---
void calibrateJoystick() {
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_YELLOW);
    tft.setTextSize(2);
    
    // Step 1: Find Center
    tft.setCursor(20, 50); tft.println("Hands OFF Stick!");
    tft.setCursor(20, 80); tft.println("Finding Center...");
    delay(2000); 
    
    // Average 10 readings
    long sumX = 0, sumY = 0;
    for(int i=0; i<10; i++) {
        sumX += analogRead(JOY_X_PIN);
        sumY += analogRead(JOY_Y_PIN);
        delay(20);
    }
    xCenter = sumX / 10;
    yCenter = sumY / 10;
    
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(20, 110); 
    tft.print("Center: "); tft.print(xCenter); tft.print(","); tft.println(yCenter);
    delay(1000);

    // Step 2: Find Range
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_YELLOW);
    tft.setCursor(20, 50); tft.println("Move Stick in");
    tft.setCursor(20, 80); tft.println("CIRCLES now!");
    
    // Reset limits to center
    xMin = xCenter; xMax = xCenter;
    yMin = yCenter; yMax = yCenter;
    
    // Record min/max for 4 seconds
    unsigned long startTime = millis();
    while (millis() - startTime < 4000) {
        int valX = analogRead(JOY_X_PIN);
        int valY = analogRead(JOY_Y_PIN);
        
        if (valX < xMin) xMin = valX;
        if (valX > xMax) xMax = valX;
        if (valY < yMin) yMin = valY;
        if (valY > yMax) yMax = valY;
        
        // Visual Progress Bar
        int bar = map(millis() - startTime, 0, 4000, 0, 300);
        tft.fillRect(10, 200, bar, 10, ILI9341_BLUE);
        delay(10);
    }
    
    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(20, 50); tft.println("Calibration Done!");
    delay(1000);
    tft.fillScreen(ILI9341_BLACK);
}

void setup() {
    Serial.begin(115200);
    
    // 1. INIT SCREEN
    tft.begin();
    tft.setRotation(1); // Landscape mode
    tft.fillScreen(ILI9341_BLACK);
    
    // 2. RUN CALIBRATION
    calibrateJoystick();
    
    // 3. DRAW UI HEADER
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 10); 
    tft.print("SAILBOT v");
    tft.println(PROTOCOL_VERSION);
    
    // 4. INIT RADIO
    tft.setCursor(10, 40); 
    tft.print("Radio... ");
    int state = radio.begin(LORA_FREQ);
    if (state == RADIOLIB_ERR_NONE) {
        tft.setTextColor(ILI9341_GREEN); 
        tft.println("OK");
        radio.setOutputPower(20);
        
        // Match boat settings for best compatibility
        radio.setSpreadingFactor(10);
        radio.setBandwidth(125.0);
        radio.setCodingRate(8);
        
        Serial.println("Radio initialized");
    } else {
        tft.setTextColor(ILI9341_RED); 
        tft.print("FAIL: "); 
        tft.println(state);
        while(1);
    }
    
    // 5. DRAW UI DASHBOARD
    tft.drawFastHLine(0, 70, 320, ILI9341_WHITE);
    tft.drawFastHLine(0, 160, 320, ILI9341_WHITE);
    
    // 6. Mode Label
    tft.setTextSize(2);
    tft.setCursor(200, 10);
    tft.setTextColor(ILI9341_CYAN);
    tft.print("MANUAL");
}

void updateUI() {
    // Only update UI every 200ms to reduce flicker
    if (millis() - lastUIUpdate < 200) return;
    lastUIUpdate = millis();
    
    // --- TOP: COMMANDS ---
    tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
    tft.setTextSize(3);
    
    tft.setCursor(10, 90);
    tft.print("RUD:");
    tft.setCursor(120, 90);
    if (txControl.rudder >= 0) tft.print(" ");
    tft.print(txControl.rudder);
    tft.print("  ");
    
    tft.setCursor(10, 125);
    tft.print("SAIL:");
    tft.setCursor(130, 125);
    tft.print(txControl.sail);
    tft.print("  ");

    // --- BOTTOM: TELEMETRY ---
    linkActive = (millis() - lastRx < 3000);
    
    if (linkActive) {
        tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
        tft.setTextSize(2);
        
        tft.setCursor(10, 180); 
        tft.print("HDG:");
        tft.setCursor(70, 180);
        tft.print(lastTelemetry.heading, 0);
        tft.print("  ");
        
        tft.setCursor(160, 180);
        tft.print("BAT:");
        tft.setCursor(220, 180);
        tft.print(lastTelemetry.battery, 1);
        tft.print("V ");
        
        tft.setCursor(10, 210);
        tft.print("WIND:");
        tft.setCursor(90, 210);
        tft.print(lastTelemetry.windAngle, 0);
        tft.print("  ");
        
    } else {
        tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
        tft.setTextSize(2);
        tft.setCursor(10, 180); 
        tft.print("NO LINK            ");
        tft.setCursor(10, 210);
        tft.print("                   ");
    }
}

void loop() {
    // 1. READ JOYSTICK
    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);
    
    // Use protocol helper functions for mapping
    txControl.rudder = mapJoystickToRudder(rawX, xCenter, xMin, xMax, CALIB_DEADZONE);
    txControl.sail = mapJoystickToSail(rawY, yCenter, yMin, yMax, CALIB_DEADZONE);
    
    // Constrain values (redundant safety check)
    txControl.rudder = constrain(txControl.rudder, -100, 100);
    txControl.sail = constrain(txControl.sail, 0, 100);

    // 2. SEND CONTROL COMMANDS (10Hz)
    if (millis() - lastTx > 100) {
        lastTx = millis();
        
        // Update checksum before sending
        txControl.updateChecksum();
        
        int state = radio.transmit((uint8_t*)&txControl, sizeof(txControl));
        
        if (state == RADIOLIB_ERR_NONE) {
            Serial.println("Control packet sent");
        } else {
            Serial.print("TX failed: ");
            Serial.println(state);
        }
        
        updateUI();
    }

    // 3. RECEIVE TELEMETRY (Non-blocking)
    // Start receive if not already receiving
    radio.startReceive();
    
    uint8_t rxBuffer[256];
    int state = radio.readData(rxBuffer, sizeof(rxBuffer));
    
    if (state == RADIOLIB_ERR_NONE) {
        // Check packet type
        PacketType pktType = getPacketType(rxBuffer);
        
        if (pktType == PKT_TELEMETRY) {
            TelemetryPacket* telem = (TelemetryPacket*)rxBuffer;
            
            if (telem->isValid()) {
                // Copy data
                lastTelemetry = *telem;
                lastRx = millis();
                
                Serial.print("Telemetry: HDG=");
                Serial.print(lastTelemetry.heading);
                Serial.print(" WIND=");
                Serial.print(lastTelemetry.windAngle);
                Serial.print(" BAT=");
                Serial.println(lastTelemetry.battery);
            } else {
                Serial.println("Telemetry checksum failed!");
            }
        }
    }
    
    // Small delay to prevent overwhelming the radio
    delay(10);
}
