Developed as part of the UCL ENGF0001 module.\<br>
Arduino-ESP connectivity, WiFi connection, and integration of software 
by Ioan Steffan Thomas.\<br>
Website interface, websocket connection and stirring subsystem code
by Sameer Kurbanov.\<br>
Heating subsystem code by Ceto Kim.\<br>
pH subsystem code by Ibraheem Siddiqui.\<br>


ESP - Arduino connectivity done with I2C protocol.\<br>
https://docs.arduino.cc/learn/communication/wire/\<br>
The data lines (SDA) and clock lines (SCL) should be 
connected via a level-shifter between Arduino and ESP.\<br>
The level-shifter should also connect the ground (GND)
pins.
ESP32 pins:\<br>
   - SDA: GPIO 21\<br>
   - SCL: GPIO 22\<br>
Arduino Uno pins:\<br>
   - SDA: SDA/A4\<br>
   - SCL: SCL/A5\<br>
The Arduino is the device given an address of 1.\<br>
