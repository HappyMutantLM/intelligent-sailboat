#include "Sensors.h"

// Initialize GPS on I2C
SensorManager::SensorManager() : _gps(&Wire) {
    _rudderOffset = 0.0;
    _sailOffset = 0.0;
    _windOffset = 0.0;
    _currentHeading = 0.0;
}

bool SensorManager::begin() {
    Wire.begin();
    
    // 1. Initialize GPS
    if (!_gps.begin(0x10)) { // 0x10 is standard I2C GPS address
        Serial.println("Error: GPS not found!");
        // We continue anyway, maybe it will wake up later
    }
    
    _gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Request RMC and GGA sentences
    _gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // 10 Hz update rate
    
    // 2. Initialize BNO085 Compass
    if (!_bno08x.begin_I2C()) {
        Serial.println("Error: BNO085 not found!");
        return false;
    }
    
    // Enable the "Rotation Vector" report at 50ms interval (20Hz)
    // specific report for heading/compass
    if (!_bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 50000)) {
        Serial.println("Could not enable Rotation Vector");
    }

    return true;
}

void SensorManager::update() {
    // 1. GPS Housekeeping
    _gps.read(); // Read new bytes
    if (_gps.newNMEAreceived()) {
        _gps.parse(_gps.lastNMEA());
    }

    // 2. Compass Housekeeping
    sh2_SensorValue_t sensorValue;
    if (_bno08x.getSensorEvent(&sensorValue)) {
        // Check if it's the rotation vector we asked for
        if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
            // Convert Quaternion to Heading (Yaw)
            float r = sensorValue.un.arvrStabilizedRV.real;
            float i = sensorValue.un.arvrStabilizedRV.i;
            float j = sensorValue.un.arvrStabilizedRV.j;
            float k = sensorValue.un.arvrStabilizedRV.k;

            // Standard conversion from Quaternion to Euler Yaw
            float siny_cosp = 2 * (r * k + i * j);
            float cosy_cosp = 1 - 2 * (j * j + k * k);
            float yaw = atan2(siny_cosp, cosy_cosp);

            // Convert radians to degrees (0-360)
            float heading = yaw * 180.0 / PI;
            if (heading < 0) heading += 360.0;
            
            _currentHeading = heading;
        }
    }
}

// --- Getters ---

float SensorManager::getLatitude() { return _gps.latitudeDegrees; }
float SensorManager::getLongitude() { return _gps.longitudeDegrees; }
bool  SensorManager::hasGPSFix() { return _gps.fix; }
float SensorManager::getHeading() { return _currentHeading; }

// --- Hardware Helpers (Multiplexer & AS5600) ---

void SensorManager::tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

float SensorManager::readAS5600() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0C); 
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 2);
    
    if (Wire.available() != 2) return -1.0; 
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();
    return ((float)((highByte << 8) | lowByte) * 360.0 / 4096.0);
}

float SensorManager::normalizeAngle(float angle) {
    while (angle < 0) angle += 360.0;
    while (angle >= 360.0) angle -= 360.0;
    return angle;
}

void SensorManager::setRudderOffset(float offsetDeg) { _rudderOffset = offsetDeg; }
void SensorManager::setSailOffset(float offsetDeg)   { _sailOffset = offsetDeg; }
void SensorManager::setWindOffset(float offsetDeg)   { _windOffset = offsetDeg; }

float SensorManager::getRudderAngle() {
    tcaSelect(MUX_CHAN_RUDDER);
    float val = readAS5600();
    return (val < 0) ? -999 : normalizeAngle(val - _rudderOffset);
}

float SensorManager::getSailAngle() {
    tcaSelect(MUX_CHAN_SAIL);
    float val = readAS5600();
    return (val < 0) ? -999 : normalizeAngle(val - _sailOffset);
}

float SensorManager::getWindAngle() {
    tcaSelect(MUX_CHAN_WIND);
    float val = readAS5600();
    return (val < 0) ? -999 : normalizeAngle(val - _windOffset);
}