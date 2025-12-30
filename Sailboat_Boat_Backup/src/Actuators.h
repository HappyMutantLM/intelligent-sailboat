#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <Arduino.h>
#include <Servo.h>

class ActuatorManager {
public:
    void begin();
    
    // Set Rudder (-45 left, 0 center, +45 right)
    void setRudder(int angle);
    
    // Set Sail (0 tight, 90 loose)
    void setSail(int angle);

private:
    Servo _rudderServo;
    Servo _sailServo;
};

#endif