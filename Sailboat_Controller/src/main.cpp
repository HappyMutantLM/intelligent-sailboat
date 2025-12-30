#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <RadioLib.h>
#include "ControllerConfig.h"

// --- OBJECTS ---
RFM95 radio = new Module(LORA_CS, LORA_IRQ, LORA_RST);
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// --- SHARED DATA STRUCTURES (Must match Boat!) ---
struct ControlPacket {
    int16_t rudder; // -45 to +45
    int16_t sail;   // 0 to 90
};

ControlPacket txPacket;
float boatHdg = 0;
float boatBatt = 0;

// --- CALIBRATION VARIABLES ---
int xMin = 0, xMax = 1023, xCenter = 512;
int yMin = 0, yMax = 1023, yCenter = 512;

// --- TIMERS ---
unsigned long lastTx = 0;
unsigned long lastRx = 0;

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
    tft.setRotation(1); // Pins at Top (Corrected Orientation)
    tft.fillScreen(ILI9341_BLACK);
    
    // 2. RUN CALIBRATION
    calibrateJoystick();
    
    // 3. DRAW UI HEADER
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 10); tft.print("SAILBOT REMOTE");
    
    // 4. INIT RADIO
    tft.setCursor(10, 40); tft.print("Radio... ");
    int state = radio.begin(LORA_FREQ);
    if (state == RADIOLIB_ERR_NONE) {
        tft.setTextColor(ILI9341_GREEN); tft.println("OK");
        radio.setOutputPower(20);
    } else {
        tft.setTextColor(ILI9341_RED); tft.print("FAIL: "); tft.println(state);
        while(1);
    }
    
    // 5. DRAW UI DASHBOARD
    tft.drawFastHLine(0, 70, 320, ILI9341_WHITE);
    tft.drawFastHLine(0, 160, 320, ILI9341_WHITE);
}

void updateUI() {
    // --- TOP: COMMANDS ---
    tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
    tft.setTextSize(3);
    
    tft.setCursor(10, 90);
    tft.print("RUD: "); tft.print(txPacket.rudder); tft.print("   ");
    
    tft.setCursor(10, 125);
    tft.print("SAIL: "); tft.print(txPacket.sail); tft.print("   ");

    // --- BOTTOM: TELEMETRY ---
    if (millis() - lastRx < 2000) {
        tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
        tft.setTextSize(2);
        tft.setCursor(10, 180); 
        tft.print("HDG: "); tft.print(boatHdg, 1); tft.print("   ");
        
        tft.setCursor(180, 180);
        tft.print("BAT: "); tft.print(boatBatt, 1); tft.println("V");
    } else {
        tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
        tft.setTextSize(2);
        tft.setCursor(10, 180); tft.print("NO LINK DETECTED   ");
    }
}

void loop() {
    // 1. READ JOYSTICK
    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);
    
    // --- RUDDER LOGIC (Split Mapping) ---
    // Maps Joystick X to -45 (Left) ... 0 ... +45 (Right)
    if (rawX < xCenter - CALIB_DEADZONE) {
        // Left Side
        txPacket.rudder = map(rawX, xMin, xCenter - CALIB_DEADZONE, -45, 0);
    } else if (rawX > xCenter + CALIB_DEADZONE) {
        // Right Side
        txPacket.rudder = map(rawX, xCenter + CALIB_DEADZONE, xMax, 0, 45);
    } else {
        // Deadzone
        txPacket.rudder = 0;
    }
    txPacket.rudder = constrain(txPacket.rudder, -45, 45);
    
    // --- SAIL LOGIC ---
    // Maps Joystick Y to 0 (Tight) ... 90 (Loose)
    if (rawY < yCenter - CALIB_DEADZONE) {
        // Up/Tight
        txPacket.sail = map(rawY, yMin, yCenter - CALIB_DEADZONE, 0, 45);
    } else if (rawY > yCenter + CALIB_DEADZONE) {
        // Down/Loose
        txPacket.sail = map(rawY, yCenter + CALIB_DEADZONE, yMax, 45, 90);
    } else {
        txPacket.sail = 45; // Center point
    }
    txPacket.sail = constrain(txPacket.sail, 0, 90);

    // 2. SEND COMMANDS (10Hz)
    if (millis() - lastTx > 100) {
        lastTx = millis();
        radio.transmit((uint8_t*)&txPacket, sizeof(txPacket));
        updateUI();
    }

    // 3. RECEIVE TELEMETRY
    // Format: "HDG,LAT,LON,BATT"
    String str;
    int state = radio.receive(str);
    if (state == RADIOLIB_ERR_NONE) {
        lastRx = millis();
        
        // Parse CSV string
        int firstComma = str.indexOf(',');
        int secondComma = str.indexOf(',', firstComma + 1);
        int thirdComma = str.indexOf(',', secondComma + 1);
        
        if (firstComma > 0) {
            String hStr = str.substring(0, firstComma);
            // Latitude and Longitude are in the middle (skipped for display)
            String bStr = str.substring(thirdComma + 1);
            
            boatHdg = hStr.toFloat();
            boatBatt = bStr.toFloat();
        }
    }
}
