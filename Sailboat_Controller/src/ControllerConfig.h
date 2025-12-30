#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#include <Arduino.h>

// -- LORA RADIO PINS (Feather M0 RFM9x) --
#define LORA_CS   8
#define LORA_RST  4
#define LORA_IRQ  3
#define LORA_FREQ 915.0

// -- JOYSTICK PINS --
#define JOY_X_PIN A0 // Rudder Control
#define JOY_Y_PIN A1 // Sail Control
#define JOY_BTN   A2 // Optional Button

// -- TFT PINS (FeatherWing) --
#define TFT_DC   10
#define TFT_CS   9
#define STMPE_CS 6 

// -- CALIBRATION DEFAULTS --
#define CALIB_DEADZONE 15

#endif