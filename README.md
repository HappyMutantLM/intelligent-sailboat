# intelligent-sailboat
Taking a Tippecanoe T50 Sloop into the 21st century

I’m creating an intelligent sailboat. I’ve build several radio controlled sailboats with 50 inch wooden hulls and carbon fiber spars. 
I’ve built these with radio control hardware. This time, I will use microprocessors. 
The goal is that this boat will be able to be sailed like a standard radio controlled sailboat, 
with control of sail angle and rudder, but also sail on its own. Meaning, a coordinates (or point on a map) 
can be entered and it will go on its own, understanding the basic mechanics of sailing. 
There will be one system for the boat, a low-profile handheld controller and then a base station that can relay directions, 
but also receive full telemetry.

Boat Hardware:
Adafruit RFM95W LoRa Radio Transceiver Breakout - 868 or 915 MHz - RadioFruit PID: 3072
Adafruit 9-DOF Orientation IMU Fusion Breakout - BNO085 (BNO080) - STEMMA QT / Qwiic PID: 4754 PowerBoost 1000 Charger - Rechargeable 5V Lipo USB Boost @ 1A - 1000C PID: 2465
Lithium Ion Battery Pack - 3.7V 6600mAh PID: 353
Adafruit Metro M4 feat. Microchip ATSAMD51 PID: 3382
Adafruit Ultimate GPS Breakout - 66 channel w/10 Hz updates - PA1616S PID: 746
Adafruit AS5600 Magnetic Angle Sensor - STEMMA QT PID: 6357 one for each of the servos and one for wind direction
Standard Size - High Torque - Metal Gear Servo PID: 1142 - one for sails one for rudder
TCA9548A I2C Multiplexer PID: 2717
Breadboard-friendly RGB Smart NeoPixel - Pack of 5 Product ID: 1312
UBEC DC/DC Step-Down (Buck) Converter - 5V @ 3A output PID: 1385
MILAPEAK 10pcs (5 Sets) 12 Positions Dual Row 600V 15A Screw Terminal Strip Blocks with Cover + 400V 15A 12 Positions Pre-Insulated Terminals Barrier Strip (Black & Red) (https://www.amazon.com/dp/B07CLW5FPS?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1)

Controller Hardware
Analog 2-axis Thumb Joystick with Select Button + Breakout Board PID: 512
TFT FeatherWing - 2.4" 320x240 Touchscreen For All Feathers - V2 PID: 3315
Lithium Ion Cylindrical Battery - 3.7v 2200mAh PID: 1781
Adafruit Feather M0 with RFM95 LoRa Radio - 900MHz - RadioFruit PID: 3178

Base Station Hardware
Raspberry Pi 4 Model B - 4 GB RAM PID: 4296
Raspberry Pi Touch Display 2 - 7" 720x1280 with Capacitive Touch PID: 6079
Adafruit RFM95W LoRa Radio Transceiver Breakout - 868 or 915 MHz - RadioFruit PID: 3072
Adafruit Feather M0 with RFM95 LoRa Radio - 900MHz - RadioFruit PID: 3178 maybe this as a dongle....

