Developed as part of the UCL ENGF0001 module.\n
Arduino-ESP connectivity, WiFi connection, and integration of software 
by Ioan Steffan Thomas.\n
Website interface, websocket connection and stirring subsystem code
by Sameer Kurbanov.\n
Heating subsystem code by Ceto Kim.\n
pH subsystem code by Ibraheem Siddiqui.\n


ESP - Arduino connectivity done with I2C protocol.\n
https://docs.arduino.cc/learn/communication/wire/\n
The data lines (SDA) and clock lines (SCL) should be 
connected via a level-shifter between Arduino and ESP.\n
The level-shifter should also connect the ground (GND)
pins.
ESP32 pins:\n
   - SDA: GPIO 21\n
   - SCL: GPIO 22\n
Arduino Uno pins:\n
   - SDA: SDA/A4\n
   - SCL: SCL/A5\n
The Arduino is the device given an address of 1.\n
