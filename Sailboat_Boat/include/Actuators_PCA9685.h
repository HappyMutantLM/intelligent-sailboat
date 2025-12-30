#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 Servo Channels
#define SERVO_CHANNEL_RUDDER 0
#define SERVO_CHANNEL_SAIL   1

class ActuatorManager {
public:
    ActuatorManager();
    void begin();

    // -100 (Left) to 100 (Right)
    void setRudder(int percentage);

    // 0 (Tight/Close-hauled) to 100 (Loose/Running)
    void setSail(int percentage);

private:
    Adafruit_PWMServoDriver _pwm;

    // Calibration: Pulse widths in microseconds
    // ADJUST THESE FOR YOUR SPECIFIC SERVOS
    const int RUDDER_MIN_US = 1000;  // Full left
    const int RUDDER_MAX_US = 2000;  // Full right
    
    const int SAIL_TIGHT_US = 1000;  // Tight (close-hauled)
    const int SAIL_LOOSE_US = 2000;  // Loose (running)
    
    // Helper function to convert microseconds to PCA9685 pulse value
    int usToPulse(int us);
};

#endif
