#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=================================");
  Serial.println("MG995R Servo Test");
  Serial.println("Tower Pro Digital Hi-Torque");
  Serial.println("=================================\n");
  
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // Important for accuracy
  pwm.setPWMFreq(50);  // 50Hz for servos
  
  delay(100);
  
  Serial.println("PCA9685 initialized!");
  Serial.println("Testing Channel 0...\n");
  
  delay(2000);
}

void loop() {
  Serial.println("Position: 1000µs (FULL LEFT)");
  pwm.writeMicroseconds(0, 1000);
  delay(2000);
  
  Serial.println("Position: 1500µs (CENTER)");
  pwm.writeMicroseconds(0, 1500);
  delay(2000);
  
  Serial.println("Position: 2000µs (FULL RIGHT)");
  pwm.writeMicroseconds(0, 2000);
  delay(2000);
  
  Serial.println("Position: 1500µs (CENTER)");
  pwm.writeMicroseconds(0, 1500);
  delay(2000);
  
  Serial.println("---\n");
}
