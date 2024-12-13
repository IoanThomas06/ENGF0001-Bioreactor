/*
 * Developed as part of the UCL ENGF0001 module.
 * Arduino-ESP connectivity and WiFi connection by Ioan Steffan Thomas.
 * Website interface and websocket connection by Sameer Kurbanov.
 */


/*
 * ESP - Arduino connectivity done with I2C protocol.
 * https://docs.arduino.cc/learn/communication/wire/
 * The data lines (SDA) and clock lines (SCL) should be 
 * connected via a level-shifter between Arduino and ESP.
 * The level-shifter should also connect the ground (GND)
 * pins.
 * ESP32 pins:
 *    - SDA: GPIO 21
 *    - SCL: GPIO 22
 * The Arduino is the device given an address of 1.
 */


#include "wifi_connection.h"
#include "data.h"
#include "websocket_client.h"
#include <Wire.h>

#define MONITOR_BAUD_RATE 115200
#define ARDUINO_ADDRESS 1


const char* websockets_server = "192.168.137.1"; // Change for IP on LAN, or host IP on Internet.
const uint16_t websockets_port = 3000; // 3000 is the local host ip address

Data bioreactor_data;
Data target_data;

void start_serial()
{
  Serial.end();
  Serial.begin(MONITOR_BAUD_RATE);
  // Waits for the serial connection to be secured.
  while (!Serial);
  // Adds initial delay so the serial monitor will receive any messages (change as needed).
  delay(1000);
}

void data_setup()
{
  target_data.ph = String(-1);
  target_data.temp = String(-1);
  target_data.rpm = String(-1);
  bioreactor_data.ph = String(-1);
  bioreactor_data.temp = String(-1);
  bioreactor_data.rpm = String(-1);
  Serial.println("Data setup successful.");
}

void wire_setup()
{
  Wire.begin();
  Serial.println("Wire serial I2C setup successful.");
}

void write_wire_data()
{
  String message = target_data.ph + "," + target_data.temp + "," + target_data.rpm + "\n";

  // Testing.
  // message = "7,25,500\n"; // Remove once target_data is being set.
  // End testing.

  Wire.beginTransmission(ARDUINO_ADDRESS);
  for (int i = 0; i < message.length(); i++)
  {
    Wire.write(message[i]);
  }
  Wire.endTransmission();
}

void read_wire_data()
{
  Wire.requestFrom(ARDUINO_ADDRESS, 15);
  String data = String("");
  int count = 0;
  while (Wire.available())
  {
    char c = Wire.read();
    if (c == ',' || c == '\n')
    {
      switch (count++){
        case 0:
          bioreactor_data.ph = data;
          break;
        case 1:
          bioreactor_data.temp = data;
          break;
        case 2:
          bioreactor_data.rpm = data;
          break;
        default:
          break;
      }
      data = String("");
    }
    else
    {
      data += c;// does this work in cpp? doesn't in c but I don't know
    }
  }
}

/*
 * get_bioreactor<> functions work given that: 
 *    0 <= ph < 10 (with maximum 3 decimal places).
 *   10 <= temp < 100 (with maximum 4 total digits).
 *    0 <= rpm < 10000 (with no decimal places).
 */

float get_bioreactor_ph()
{
  return bioreactor_data.ph.toInt() / pow(10.0, bioreactor_data.ph.length() - 1.0);
}

float get_bioreactor_temp()
{
  return bioreactor_data.temp.toInt() / pow(10.0, bioreactor_data.ph.length() - 2.0);
}

int get_bioreactor_rpm()
{
  return bioreactor_data.rpm.toInt();
}

/*
 * set_<> functions work given that: 
 *    0 <= ph < 10 (with maximum 3 decimal places).
 *   10 <= temp < 100 (with maximum 4 total digits).
 *    0 <= rpm < 10000 (with no decimal places).
 */

void set_ph(float ph)
{
  target_data.ph = String((int)round(ph * 1000));
}

void set_temp(float temp)
{
  target_data.temp = String((int)round(temp * 100));
}

void set_rpm(int rpm)
{
  target_data.rpm = String(rpm);
}

void handleIncomingMessage(uint8_t *payload) 
{
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, payload);

  if (doc.containsKey("heat")) 
  {
    set_temp(doc["heat"]);
  }

  if (doc.containsKey("pH")) 
  {
    set_ph(doc["pH"]);
  }

  if (doc.containsKey("rpm")) 
  {
    set_rpm(doc["rpm"]);
  }
}

void setup() 
{
  start_serial();
  
  data_setup();
  connect_to_wifi();
  setupWebSocket(websockets_server, websockets_port);
  wire_setup();

  Serial.println("Setup successful.");
  delay(3000);
}

void loop() 
{
  maintain_wifi_connection();
  read_wire_data();
  write_wire_data();

  webSocket.loop();
  sendData(bioreactor_data);

  // Testing.
  Serial.println("Target pH:   " + target_data.ph);
  Serial.println("Target Temp: " + target_data.temp);
  Serial.println("Target RPM:  " + target_data.rpm);

  Serial.println("Bioreactor pH:   " + String(get_bioreactor_ph()));
  Serial.println("Bioreactor Temp: " + String(get_bioreactor_temp()));
  Serial.println("Bioreactor RPM:  " + String(get_bioreactor_rpm()));
  // End testing.

  delay(100);
}
