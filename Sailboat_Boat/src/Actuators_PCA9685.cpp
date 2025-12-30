#include "Actuators.h"

ActuatorManager::ActuatorManager() {
    // Constructor - PCA9685 object is initialized with default I2C address (0x40)
}

void ActuatorManager::begin() {
    // Initialize PCA9685
    _pwm.begin();
    
    // Set PWM frequency to 50Hz (standard for analog servos)
    // Digital servos can often handle 100Hz, but 50Hz is universally safe
    _pwm.setPWMFreq(50);
    
    // Small delay to let the PCA9685 stabilize
    delay(10);
    
    // Move servos to safe starting positions
    setRudder(0);   // Centered
    setSail(50);    // Mid-position
    
    Serial.println("PCA9685 Servo Driver initialized");
    Serial.println("Servos set to safe start positions");
}

void ActuatorManager::setRudder(int percentage) {
    // Constrain input to valid range
    percentage = constrain(percentage, -100, 100);
    
    // Map percentage to pulse width in microseconds
    int pulseWidth;
    if (percentage < 0) {
        // Left: Map -100 to 0 → RUDDER_MIN_US to center
        pulseWidth = map(percentage, -100, 0, RUDDER_MIN_US, (RUDDER_MIN_US + RUDDER_MAX_US) / 2);
    } else {
        // Right: Map 0 to 100 → center to RUDDER_MAX_US
        pulseWidth = map(percentage, 0, 100, (RUDDER_MIN_US + RUDDER_MAX_US) / 2, RUDDER_MAX_US);
    }
    
    // Convert to PCA9685 pulse value and send
    int pulse = usToPulse(pulseWidth);
    _pwm.setPWM(SERVO_CHANNEL_RUDDER, 0, pulse);
    
    // Debug output (comment out after testing)
    // Serial.print("Rudder: "); Serial.print(percentage); 
    // Serial.print("% = "); Serial.print(pulseWidth); Serial.println("us");
}

void ActuatorManager::setSail(int percentage) {
    // Constrain input to valid range
    percentage = constrain(percentage, 0, 100);
    
    // Map percentage to pulse width
    // 0 = tight (close-hauled), 100 = loose (running)
    int pulseWidth = map(percentage, 0, 100, SAIL_TIGHT_US, SAIL_LOOSE_US);
    
    // Convert to PCA9685 pulse value and send
    int pulse = usToPulse(pulseWidth);
    _pwm.setPWM(SERVO_CHANNEL_SAIL, 0, pulse);
    
    // Debug output (comment out after testing)
    // Serial.print("Sail: "); Serial.print(percentage); 
    // Serial.print("% = "); Serial.print(pulseWidth); Serial.println("us");
}

int ActuatorManager::usToPulse(int us) {
    // PCA9685 has 12-bit resolution (0-4095)
    // At 50Hz, each cycle is 20,000 microseconds
    // pulse = (microseconds * 4096) / 20000
    return (us * 4096) / 20000;
}
