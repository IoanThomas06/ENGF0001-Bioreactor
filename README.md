Developed as part of the UCL ENGF0001 module.  
Arduino-ESP connectivity, WiFi connection, and integration of software 
by Ioan Steffan Thomas.  
Website interface, websocket connection and stirring subsystem code
by Sameer Kurbanov.  
Heating subsystem code by Ceto Kim.  
pH subsystem code by Ibraheem Siddiqui.  


ESP - Arduino connectivity done with [I2C protocol](https://docs.arduino.cc/learn/communication/wire/).    
The data lines (SDA) and clock lines (SCL) should be 
connected via a level-shifter between Arduino and ESP.  
The level-shifter should also connect the ground (GND)
pins.  

ESP32 pins:
   - SDA: GPIO 21  
   - SCL: GPIO 22

Arduino Uno pins:
   - SDA: SDA/A4\  
   - SCL: SCL/A5\  

The Arduino is the device given an address of 1.

