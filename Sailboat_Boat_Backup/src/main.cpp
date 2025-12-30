#include <Arduino.h>
#include "Sensors.h"
#include "Actuators.h"
#include "Navigation.h"
#include "Comms.h"
#include <Adafruit_NeoPixel.h>

// --- CONFIGURATION ---
#define NEOPIXEL_PIN 5
#define NUM_PIXELS   1

// --- OBJECTS ---
SensorManager sensors;
ActuatorManager actuators;
NavigationManager nav;
CommsManager comms;
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// --- STATE VARIABLES ---
unsigned long lastTelemetry = 0;
unsigned long lastCmdReceived = 0;

void setup() {
  Serial.begin(115200);

  Serial.println("\n--- SAILBOT BOOT SEQUENCE ---");

  // 1. INIT NEOPIXEL (Red = Booting)
  strip.begin();
  strip.setBrightness(50);
  strip.setPixelColor(0, strip.Color(255, 0, 0)); 
  strip.show();

  // 2. INIT MODULES
  if (sensors.begin()) Serial.println("   [OK] Sensors");
  else                 Serial.println("   [FAIL] Sensors");

  actuators.begin();   
  Serial.println("   [OK] Actuators");
  
  if (comms.begin())   Serial.println("   [OK] Radio");
  else                 Serial.println("   [FAIL] Radio");

  Serial.println("--- SYSTEM READY ---");
  
  // 3. SET READY STATUS (Green)
  strip.setPixelColor(0, strip.Color(0, 255, 0)); 
  strip.show();
}

void loop() {
  // 1. UPDATE SENSORS
  sensors.update();

  // 2. CHECK RADIO FOR COMMANDS
  ControlPacket rxCmd;
  
  if (comms.receiveControl(rxCmd)) {
      lastCmdReceived = millis();

      // Visual Confirm: Short Blue Flash
      strip.setPixelColor(0, strip.Color(0, 0, 255)); 
      strip.show();

      // EXECUTE COMMANDS
      actuators.setRudder(rxCmd.rudder);
      actuators.setSail(rxCmd.sail);
      
      // Debug Print
      Serial.print("RX CMD -> Rudder: "); Serial.print(rxCmd.rudder);
      Serial.print(" | Sail: "); Serial.println(rxCmd.sail);
  }

  // Restore Green LED if no radio packet this cycle
  if (millis() - lastCmdReceived > 100) {
      strip.setPixelColor(0, strip.Color(0, 255, 0)); 
      strip.show();
  }

  // 3. SEND TELEMETRY (Every 1 second)
  if (millis() - lastTelemetry > 1000) {
      lastTelemetry = millis();
      
      TelemetryPacket packet;
      packet.heading = sensors.getHeading();
      packet.lat = sensors.getLat();
      packet.lon = sensors.getLon();
      packet.battery = sensors.getBatteryVoltage();
      
      comms.sendTelemetry(packet);
      
      // Console Heartbeat
      Serial.print("STATUS -> HDG: "); Serial.print(packet.heading, 1);
      Serial.print(" | Batt: "); Serial.print(packet.battery, 2);
      Serial.println("V");
  }
}