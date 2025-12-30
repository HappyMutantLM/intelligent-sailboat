#include "Sensors.h"
#include <Wire.h>

// --- HARDWARE CONFIG ---
#define TCAADDR 0x70      // Multiplexer Address
#define AS5600_ADDR 0x36  // Encoder Address

// --- HELPER: Switch Multiplexer Channels ---
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// Constructor
SensorManager::SensorManager() : _gps(&Serial1) { 
    _currentHeading = 0.0;
}

bool SensorManager::begin() {
    Serial.println("   ...Starting Wire");
    Wire.begin();
    Wire.setClock(100000); 

    // 1. INIT GPS
    Serial.println("   ...Starting GPS");
    _gps.begin(9600);
    _gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Ask for location + heading data
    _gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // 1 update per second

    // 2. INIT MULTIPLEXER (Clear the bus)
    Serial.println("   ...Clearing Mux");
    tcaselect(7); 
    delay(50);
    
    return true;
}

void SensorManager::update() {
    // --- READ GPS ---
    _gps.read();
    if (_gps.newNMEAreceived()) {
        _gps.parse(_gps.lastNMEA());
        
        // LOGIC: Use GPS 'Angle' as Heading
        if (_gps.fix) {
            _currentHeading = _gps.angle;
        }
    }
}

float SensorManager::getHeading() { return _currentHeading; }

// --- READ ENCODERS (With Noise Filter) ---
float readAS5600() {
    Wire.beginTransmission(AS5600_ADDR); 
    Wire.write(0x0C); // Register: Raw Angle
    Wire.endTransmission();
    
    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() >= 2) {
        uint16_t highbyte = Wire.read();
        uint16_t lowbyte = Wire.read();
        
        // MASK NOISE: Filter top 4 bits
        uint16_t rawAngle = ((highbyte & 0x0F) << 8) | lowbyte;
        
        // Scale 0-4095 to 0-360
        return (rawAngle * 360.0) / 4096.0;
    }
    return -1.0; 
}

float SensorManager::getRudderAngle() {
    tcaselect(0); 
    float angle = readAS5600(); 
    tcaselect(7); 
    return angle; 
}

float SensorManager::getSailAngle() {
    tcaselect(1); 
    float angle = readAS5600(); 
    tcaselect(7); 
    return angle;
}

float SensorManager::getBatteryVoltage() {
    // Feather M0 measures battery on Pin 9 (A7)
    // Voltage is divided by 2, so we multiply back.
    float v = analogRead(9); 
    v *= 2;    
    v *= 3.3;  
    v /= 1024; 
    return v;
}

float SensorManager::getLat() { return _gps.latitudeDegrees; }
float SensorManager::getLon() { return _gps.longitudeDegrees; }