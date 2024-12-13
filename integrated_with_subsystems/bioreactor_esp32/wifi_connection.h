#include <WiFi.h>

#ifndef WIFI_CONNECTION_H
#define WIFI_CONNECTION_H

// Configure for either shared LAN with the interface, 
// or for the Internet to connect to a different LAN.
// For example: https://support.microsoft.com/en-us/windows/use-your-windows-pc-as-a-mobile-hotspot-c89b0fad-72d5-41e8-f7ea-406ad9036b85
// Using a Windows hotspot to creat a LAN is how this has been used for the demonstration:
// Settings > Network & Internet > Mobile Hotspot > Toggle On/Off
const char *ssid = "FUCK 4738";  
const char *password = "$8S87o06";

void connect_to_wifi()
{
  int counter = 0;

  Serial.println();
  Serial.print("Connecting to network: ");
  Serial.println(ssid);
  WiFi.disconnect(true);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
    // Reset board after 30 seconds, if not connected.
    if (counter++ >= 60) 
    {
      ESP.restart();
    }
  }

  Serial.println(" ");
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void maintain_wifi_connection()
{
  if (WiFi.status() != WL_CONNECTED) 
  {
    int counter = 0;

    Serial.println("Wifi connection lost.");
    Serial.print("Attempting to reconnect.");
    
    WiFi.begin(ssid);
    while (WiFi.status() != WL_CONNECTED) 
    {
      delay(500);
      Serial.println(".");
      // Reset board after 15 seconds, if not connected.
      if (counter++ >= 30) 
      {
        ESP.restart();
      }
    }
    Serial.println(" ");
  }
}

#endif